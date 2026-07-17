package mchorse.bbs_mod.cubic.ik.solver;

import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * Damped-least-squares IK over channel angles — ONE solver for any chain
 * length, replacing the analytic/FABRIK/CCD fork of the old position-level
 * solver. Modeled on Blender's behavior, implemented from the standard DLS
 * literature (Buss).
 *
 * <p>Each iteration builds the Jacobian of the effector position over every
 * unlocked channel (column = {@code axis × (effector − pivot)}, the axes
 * straight from the ZYX gimbal frames — see {@link IKChain#channelAxis}),
 * solves the damped normal equations {@code (J·W²·Jᵀ + λ²I)·y = error} (a 3×3
 * system — one positional effector), steps the angles by {@code W²·Jᵀ·y},
 * clamps them into their per-axis limits, and re-poses the chain. Damping
 * makes the near-singular configurations (straight chain, gimbal-aligned
 * axes) step small and stable instead of exploding — the reason a single
 * solver can cover every length.
 *
 * <p>Locks, limits and stiffness live per axis on {@link IKJoint}: a locked
 * axis never enters the Jacobian and keeps its FK value; a limited axis is
 * clamped every iteration, so the solve routes the remaining error through
 * the free joints; stiffness scales an axis's willingness to move.
 *
 * <p>The pole is Blender's: the whole chain twists about the root→goal line
 * until the bend plane contains the pole target, plus the pole angle. Both
 * root and goal sit ON that line, so the twist never disturbs how close the
 * effector is to the goal — it is applied before the iterations (so they
 * start from the predictable side) and re-applied exactly after them.
 *
 * <p>Everything is radians and world (model) units; the solve is stateless —
 * capture, solve, write back, every frame.
 */
public final class IKChainSolver
{
    private static final float EPS = 1.0e-6f;

    /** Iteration step cap, radians — keeps a damped-but-large step from overshooting. */
    private static final float MAX_STEP = 0.35F;

    /** Steps smaller than this on every channel mean the solve has stalled (limits, locks, degeneracy). */
    private static final float STALL_STEP = 1.0e-6f;

    /**
     * @param maxIterations DLS iteration cap; ~64 covers long chains comfortably.
     * @param tolerance world-units effector error that counts as reached.
     * @param dampingRatio λ as a fraction of the chain's total length; the damping
     * grows with the Jacobian's scale so behavior is rig-size independent.
     */
    public record Params(int maxIterations, float tolerance, float dampingRatio)
    {
        public static final Params DEFAULT = new Params(64, 1.0e-4f, 0.1F);
    }

    /** @param error final effector-to-goal distance, world units. */
    public record Result(boolean reached, float error, int iterations)
    {
    }

    private IKChainSolver()
    {
    }

    /**
     * Solves {@code chain} towards {@code goal}, mutating the joints' angles.
     * {@code polePoint} may be {@code null} (no pole constraint);
     * {@code poleAngle} is radians about the root→goal line.
     */
    public static Result solve(IKChain chain, Vector3f goal, Vector3f polePoint, float poleAngle, Params params)
    {
        chain.forward();

        if (polePoint != null)
        {
            applyPole(chain, goal, polePoint, poleAngle);
        }

        float damping = params.dampingRatio() * chain.totalLength();
        float lambdaSq = damping * damping;
        int columns = countColumns(chain);
        int iterations = 0;
        float error = chain.effector.distance(goal);

        while (iterations < params.maxIterations() && error > params.tolerance() && columns > 0)
        {
            iterations++;

            if (!step(chain, goal, lambdaSq))
            {
                break;
            }

            error = chain.effector.distance(goal);
        }

        if (polePoint != null)
        {
            /* Exact bend-plane snap; root and goal are on the twist line, so the
             * effector's distance to the goal is untouched. */
            applyPole(chain, goal, polePoint, poleAngle);
            error = chain.effector.distance(goal);
        }

        return new Result(error <= params.tolerance(), error, iterations);
    }

    /**
     * Soft-reach goal preprocessor: maps the target onto an effective distance
     * that approaches the chain's full reach asymptotically (C1), so the limb
     * eases into extension instead of snapping straight — same falloff the old
     * solver used. {@code softness} is the fraction of the reach the falloff
     * spans; 0 = hard clamp at full reach. Returns the adjusted goal.
     */
    public static Vector3f softGoal(IKChain chain, Vector3f target, float softness)
    {
        Vector3f root = chain.joints[0].startPosition;
        Vector3f goal = new Vector3f(target);
        float total = chain.totalLength();
        float distance = root.distance(target);

        if (distance < EPS || total < EPS)
        {
            return goal;
        }

        Vector3f direction = new Vector3f(target).sub(root).div(distance);

        if (softness > EPS)
        {
            float soft = Math.min(softness, 1F) * total;
            float hardRange = total - soft;

            if (distance > hardRange)
            {
                float effective = total - soft * (float) Math.exp(-(distance - hardRange) / soft);

                goal.set(root).fma(Math.min(effective, total), direction);
            }
        }
        else if (distance > total)
        {
            goal.set(root).fma(total, direction);
        }

        return goal;
    }

    /**
     * Blender's pole: twist the whole chain about the root→goal line so the
     * bend plane (root → first interior joint) contains the pole target, then
     * roll by {@code poleAngle}. Skipped when the geometry defines no plane —
     * a chain with no interior joint, a goal on the root, or a bend or pole
     * sitting exactly on the twist line (the straight-chain degeneracy; a
     * pre-bent rest pose is what avoids it, as in Blender).
     */
    public static void applyPole(IKChain chain, Vector3f goal, Vector3f polePoint, float poleAngle)
    {
        if (chain.joints.length < 2)
        {
            return;
        }

        Vector3f root = chain.joints[0].position;
        Vector3f axis = new Vector3f(goal).sub(root);

        if (axis.lengthSquared() < EPS * EPS)
        {
            return;
        }

        axis.normalize();

        Vector3f bend = perpendicular(new Vector3f(chain.joints[1].position).sub(root), axis);
        Vector3f pole = perpendicular(new Vector3f(polePoint).sub(root), axis);

        if (bend == null || pole == null)
        {
            return;
        }

        float twist = (float) Math.atan2(new Vector3f(bend).cross(pole).dot(axis), bend.dot(pole)) + poleAngle;

        if (Math.abs(twist) < EPS)
        {
            return;
        }

        chain.rotateJointWorld(0, new Quaternionf().rotationAxis(twist, axis.x, axis.y, axis.z));
    }

    /** How many channels can move at all — zero means there is nothing to solve with. */
    private static int countColumns(IKChain chain)
    {
        int columns = 0;

        for (IKJoint joint : chain.joints)
        {
            for (int c = 0; c < 3; c++)
            {
                if (!joint.locked[c] && joint.weight(c) > 0F)
                {
                    columns++;
                }
            }
        }

        return columns;
    }

    /** One damped-least-squares iteration; false when the step stalls. */
    private static boolean step(IKChain chain, Vector3f goal, float lambdaSq)
    {
        IKJoint[] joints = chain.joints;
        int n = joints.length;
        Vector3f error = new Vector3f(goal).sub(chain.effector);

        /* Weighted Jacobian columns (w · axis × (effector − pivot)), and
         * A = J·Jᵀ + λ²I accumulated from them (3×3, symmetric). */
        float[] columns = new float[n * 9];
        float a00 = lambdaSq, a01 = 0F, a02 = 0F;
        float a11 = lambdaSq, a12 = 0F;
        float a22 = lambdaSq;

        Vector3f axis = new Vector3f();

        for (int i = 0; i < n; i++)
        {
            IKJoint joint = joints[i];

            for (int c = 0; c < 3; c++)
            {
                float w = joint.locked[c] ? 0F : joint.weight(c);

                if (w <= 0F)
                {
                    continue;
                }

                chain.channelAxis(i, c, axis);

                Vector3f column = new Vector3f(chain.effector).sub(joint.position);

                axis.cross(column, column).mul(w);

                int at = (i * 3 + c) * 3;

                columns[at] = column.x;
                columns[at + 1] = column.y;
                columns[at + 2] = column.z;

                a00 += column.x * column.x;
                a01 += column.x * column.y;
                a02 += column.x * column.z;
                a11 += column.y * column.y;
                a12 += column.y * column.z;
                a22 += column.z * column.z;
            }
        }

        Vector3f y = solveSymmetric3(a00, a01, a02, a11, a12, a22, error);

        if (y == null)
        {
            return false;
        }

        /* Δθ = W·Jᵀ·y (the second W of W²·Ĵᵀ·y — the first is inside the stored
         * columns), clamped per channel; apply and re-pose. */
        float largest = 0F;

        for (int i = 0; i < n; i++)
        {
            IKJoint joint = joints[i];

            for (int c = 0; c < 3; c++)
            {
                float w = joint.locked[c] ? 0F : joint.weight(c);

                if (w <= 0F)
                {
                    continue;
                }

                int at = (i * 3 + c) * 3;
                float delta = w * (columns[at] * y.x + columns[at + 1] * y.y + columns[at + 2] * y.z);

                delta = Math.max(-MAX_STEP, Math.min(MAX_STEP, delta));
                largest = Math.max(largest, Math.abs(delta));

                IKJoint.set(joint.angles, c, IKJoint.get(joint.angles, c) + delta);
            }

            joint.clampLimits();
        }

        chain.forward();

        return largest > STALL_STEP;
    }

    /** Solves the symmetric 3×3 system A·y = b by Cramer; null when A is singular. */
    private static Vector3f solveSymmetric3(float a00, float a01, float a02, float a11, float a12, float a22, Vector3f b)
    {
        float c00 = a11 * a22 - a12 * a12;
        float c01 = a02 * a12 - a01 * a22;
        float c02 = a01 * a12 - a02 * a11;
        float det = a00 * c00 + a01 * c01 + a02 * c02;

        if (Math.abs(det) < 1.0e-12f)
        {
            return null;
        }

        float c11 = a00 * a22 - a02 * a02;
        float c12 = a01 * a02 - a00 * a12;
        float c22 = a00 * a11 - a01 * a01;

        return new Vector3f(
            (c00 * b.x + c01 * b.y + c02 * b.z) / det,
            (c01 * b.x + c11 * b.y + c12 * b.z) / det,
            (c02 * b.x + c12 * b.y + c22 * b.z) / det
        );
    }

    /** {@code v} minus its component along unit {@code axis}, normalized; {@code null} when degenerate. */
    private static Vector3f perpendicular(Vector3f v, Vector3f axis)
    {
        v.fma(-v.dot(axis), axis);

        return v.lengthSquared() < EPS * EPS ? null : v.normalize();
    }
}
