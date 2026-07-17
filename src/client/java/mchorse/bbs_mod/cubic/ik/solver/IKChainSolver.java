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
                if (movableWeight(joint, c) > 0F)
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
        float errorBefore = error.length();

        /* Weighted Jacobian columns (w · axis × (effector − pivot)). A channel
         * pinned by its limits (min == max) can never move — it never enters. */
        float[] columns = new float[n * 9];
        boolean[] frozen = new boolean[n * 3];
        Vector3f axis = new Vector3f();

        for (int i = 0; i < n; i++)
        {
            IKJoint joint = joints[i];

            for (int c = 0; c < 3; c++)
            {
                float w = movableWeight(joint, c);

                if (w <= 0F)
                {
                    frozen[i * 3 + c] = true;
                    continue;
                }

                chain.channelAxis(i, c, axis);

                Vector3f column = new Vector3f(chain.effector).sub(joint.position);

                axis.cross(column, column).mul(w);

                int at = (i * 3 + c) * 3;

                columns[at] = column.x;
                columns[at + 1] = column.y;
                columns[at + 2] = column.z;
            }
        }

        float[] saved = new float[n * 3];

        for (int i = 0; i < n; i++)
        {
            for (int c = 0; c < 3; c++)
            {
                saved[i * 3 + c] = IKJoint.get(joints[i].angles, c);
            }
        }

        /* Phase 1: the full damped Gauss-Newton step. */
        float[] deltas = solveDeltas(chain, columns, frozen, error, lambdaSq);

        if (deltas == null)
        {
            return false;
        }

        float largest = applyDeltas(chain, deltas, 1F);

        chain.forward();

        if (chain.effector.distance(goal) <= errorBefore)
        {
            return largest > STALL_STEP;
        }

        /* Phase 2 (active set): when the limit clamp CUT part of that step, what
         * was applied is no longer a descent direction — the free channels moved
         * by amounts agreed with motion the clamp then denied. Freeze the cut
         * channels and re-solve: the step redistributes over what can actually
         * move and descends again, which is how a limited elbow hands the rest
         * of the reach to the free joints instead of stalling the whole solve. */
        boolean cut = false;

        for (int i = 0; i < n; i++)
        {
            for (int c = 0; c < 3; c++)
            {
                int at = i * 3 + c;

                if (!frozen[at] && deltas[at] != 0F && Math.abs(IKJoint.get(joints[i].angles, c) - (saved[at] + deltas[at])) > 1.0e-7f)
                {
                    frozen[at] = true;
                    cut = true;
                }
            }
        }

        restoreAngles(chain, saved);

        if (cut)
        {
            deltas = solveDeltas(chain, columns, frozen, error, lambdaSq);

            if (deltas != null)
            {
                largest = applyDeltas(chain, deltas, 1F);
                chain.forward();

                if (chain.effector.distance(goal) <= errorBefore)
                {
                    return largest > STALL_STEP;
                }

                restoreAngles(chain, saved);
            }
        }

        /* Phase 3 (backtracking): the step overshot — near-degenerate goals make
         * a damped step OSCILLATE, and running out of iterations mid-oscillation
         * hands each frame a random phase of it (the chain twitches while the
         * target barely moves). Retry at half scale a few times; if even the
         * smallest scale is worse, this is the local best — restore and stop.
         * Error is monotonically non-increasing, frames stay steady. */
        if (deltas != null)
        {
            float scale = 0.5F;

            for (int attempt = 0; attempt < 3; attempt++)
            {
                largest = applyDeltas(chain, deltas, scale);
                chain.forward();

                if (chain.effector.distance(goal) <= errorBefore)
                {
                    return largest > STALL_STEP;
                }

                restoreAngles(chain, saved);
                scale *= 0.5F;
            }
        }

        chain.forward();

        return false;
    }

    /** How much this channel may move at all: 0 when locked, weightless, or pinned by min == max limits. */
    private static float movableWeight(IKJoint joint, int axis)
    {
        if (joint.locked[axis] || (joint.limited[axis] && joint.limitMin[axis] >= joint.limitMax[axis]))
        {
            return 0F;
        }

        return joint.weight(axis);
    }

    /**
     * Solves the damped normal equations over the unfrozen columns and returns
     * the per-channel deltas {@code W·Jᵀ·y} (the second W of {@code W²·Ĵᵀ·y} —
     * the first is inside the stored columns), clamped to {@link #MAX_STEP};
     * {@code null} when the system is singular.
     */
    private static float[] solveDeltas(IKChain chain, float[] columns, boolean[] frozen, Vector3f error, float lambdaSq)
    {
        IKJoint[] joints = chain.joints;
        int n = joints.length;
        float a00 = lambdaSq, a01 = 0F, a02 = 0F;
        float a11 = lambdaSq, a12 = 0F;
        float a22 = lambdaSq;

        for (int at = 0; at < n * 3; at++)
        {
            if (frozen[at])
            {
                continue;
            }

            float x = columns[at * 3];
            float y = columns[at * 3 + 1];
            float z = columns[at * 3 + 2];

            a00 += x * x;
            a01 += x * y;
            a02 += x * z;
            a11 += y * y;
            a12 += y * z;
            a22 += z * z;
        }

        Vector3f y = solveSymmetric3(a00, a01, a02, a11, a12, a22, error);

        if (y == null)
        {
            return null;
        }

        float[] deltas = new float[n * 3];

        for (int i = 0; i < n; i++)
        {
            IKJoint joint = joints[i];

            for (int c = 0; c < 3; c++)
            {
                int at = i * 3 + c;

                if (frozen[at])
                {
                    continue;
                }

                float w = movableWeight(joint, c);
                float delta = w * (columns[at * 3] * y.x + columns[at * 3 + 1] * y.y + columns[at * 3 + 2] * y.z);

                deltas[at] = Math.max(-MAX_STEP, Math.min(MAX_STEP, delta));
            }
        }

        return deltas;
    }

    /** Applies {@code scale}-sized deltas onto the current angles; returns the largest applied delta. */
    private static float applyDeltas(IKChain chain, float[] deltas, float scale)
    {
        float largest = 0F;

        for (int i = 0; i < chain.joints.length; i++)
        {
            IKJoint joint = chain.joints[i];

            for (int c = 0; c < 3; c++)
            {
                float delta = deltas[i * 3 + c] * scale;

                if (delta != 0F)
                {
                    largest = Math.max(largest, Math.abs(delta));
                    IKJoint.set(joint.angles, c, IKJoint.get(joint.angles, c) + delta);
                }
            }

            joint.clampLimits();
        }

        return largest;
    }

    private static void restoreAngles(IKChain chain, float[] saved)
    {
        for (int i = 0; i < chain.joints.length; i++)
        {
            IKJoint joint = chain.joints[i];

            for (int c = 0; c < 3; c++)
            {
                IKJoint.set(joint.angles, c, saved[i * 3 + c]);
            }
        }
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
