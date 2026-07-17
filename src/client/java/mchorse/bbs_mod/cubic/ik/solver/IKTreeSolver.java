package mchorse.bbs_mod.cubic.ik.solver;

import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * Damped-least-squares IK over channel angles for a whole {@link IKTree} —
 * ONE solver for any chain length and any number of merged chains, modeled on
 * Blender's behavior, implemented from the standard DLS literature (Buss).
 *
 * <p>Each iteration builds the Jacobian of every effector position over every
 * unlocked channel (a column carries, per effector it moves,
 * {@code axis × (effector − pivot)} — axes straight from the ZYX gimbal
 * frames, see {@link IKTree#channelAxis}), solves the damped normal equations
 * {@code (J·W²·Jᵀ + λ²I)·y = error} — a 3k×3k symmetric positive-definite
 * system (k = effectors), by Cholesky — steps the angles by {@code W²·Jᵀ·y},
 * clamps them into their per-axis limits, and re-poses the tree. Shared bones
 * NEGOTIATE between goals through the shared columns; effector weights scale
 * their rows, so a lighter goal yields where goals conflict.
 *
 * <p>The step is MONOTONE in the combined weighted error: a step that made it
 * worse is retried with the limit-cut channels frozen (active set — a clamped
 * step is no longer a descent direction), then backtracked at halved scales;
 * the cap on a large step scales the WHOLE delta vector, never per-channel
 * (a per-channel clamp degenerates the direction into a vector of signs).
 *
 * <p>Locks, limits and stiffness live per axis on {@link IKJoint}. The pole is
 * Blender's root twist about the root→goal line; on a SINGLE chain it is
 * applied before the iterations and re-applied exactly after them (root and
 * goal sit on the line, so the twist cannot disturb that chain's reach). On a
 * merged tree poles are pre-applied only — an exact post-twist of one chain
 * would move the OTHER chains' effectors — and the goal pre-alignment and
 * extension-break degeneracy cures stay single-chain-only for the same
 * reason. Everything is radians and world units; the solve is stateless.
 */
public final class IKTreeSolver
{
    private static final float EPS = 1.0e-6f;

    /** Iteration step cap, radians — keeps a damped-but-large step from overshooting. */
    private static final float MAX_STEP = 0.35F;

    /** Steps smaller than this on every channel mean the solve has stalled (limits, locks, degeneracy). */
    private static final float STALL_STEP = 1.0e-6f;

    /**
     * @param maxIterations DLS iteration cap; ~64 covers long chains comfortably.
     * @param tolerance world-units effector error that counts as reached.
     * @param dampingRatio λ as a fraction of the tree's largest chain reach; the
     * damping grows with the Jacobian's scale so behavior is rig-size independent.
     */
    public record Params(int maxIterations, float tolerance, float dampingRatio)
    {
        public static final Params DEFAULT = new Params(64, 1.0e-4f, 0.1F);
    }

    /**
     * A chain's pole inside the tree: twist about {@code rootJoint}'s
     * root→goal line towards {@code polePoint}, offset by {@code poleAngle}.
     */
    public record Pole(int rootJoint, Vector3f polePoint, float poleAngle)
    {
    }

    /** @param error final LARGEST effector-to-goal distance, world units. */
    public record Result(boolean reached, float error, int iterations)
    {
    }

    private IKTreeSolver()
    {
    }

    /**
     * Solves the tree towards its effectors' goals, mutating the joints'
     * angles. {@code poles} may be null/empty; each entry steers one chain's
     * bend. Effector goals and weights are read from the tree.
     */
    public static Result solve(IKTree tree, Pole[] poles, Params params)
    {
        tree.forward();

        boolean single = tree.effectors.length == 1;

        if (single)
        {
            alignToGoal(tree, 0);
        }

        if (poles != null)
        {
            for (Pole pole : poles)
            {
                if (pole != null && pole.polePoint() != null)
                {
                    applyPole(tree, pole);
                }
            }
        }

        if (single)
        {
            breakExtension(tree, 0, firstPole(poles));
        }

        float damping = params.dampingRatio() * largestReach(tree);
        float lambdaSq = damping * damping;
        int columns = countColumns(tree);
        int iterations = 0;
        float error = combinedError(tree);

        while (iterations < params.maxIterations() && worstError(tree) > params.tolerance() && columns > 0)
        {
            iterations++;

            if (!step(tree, lambdaSq))
            {
                break;
            }

            error = combinedError(tree);
        }

        if (single && poles != null)
        {
            /* Exact bend-plane snap; root and goal sit on the twist line, so the
             * single chain's reach is untouched. */
            for (Pole pole : poles)
            {
                if (pole != null && pole.polePoint() != null)
                {
                    applyPole(tree, pole);
                }
            }
        }

        float worst = worstError(tree);

        return new Result(worst <= params.tolerance(), worst, iterations);
    }

    /**
     * Soft-reach goal preprocessor: maps a target onto an effective distance
     * that approaches the chain's full reach asymptotically (C1), so the limb
     * eases into extension instead of snapping straight. {@code softness} is
     * the fraction of the reach the falloff spans; 0 = hard clamp at full
     * reach. Returns the adjusted goal.
     */
    public static Vector3f softGoal(Vector3f rootPosition, float reach, Vector3f target, float softness)
    {
        Vector3f goal = new Vector3f(target);
        float distance = rootPosition.distance(target);

        if (distance < EPS || reach < EPS)
        {
            return goal;
        }

        Vector3f direction = new Vector3f(target).sub(rootPosition).div(distance);

        if (softness > EPS)
        {
            float soft = Math.min(softness, 1F) * reach;
            float hardRange = reach - soft;

            if (distance > hardRange)
            {
                float effective = reach - soft * (float) Math.exp(-(distance - hardRange) / soft);

                goal.set(rootPosition).fma(Math.min(effective, reach), direction);
            }
        }
        else if (distance > reach)
        {
            goal.set(rootPosition).fma(reach, direction);
        }

        return goal;
    }

    /**
     * Pre-aligns a single chain towards a goal in the BACK half-space: when the
     * goal sits more than 90° away from the current root→effector direction,
     * the chain's root joint rigidly turns the branch by the EXCESS over 90°
     * (zero at the boundary — poses stay continuous in the goal; 90° at the
     * exact antipode). Interior FK bends and twists ride along untouched.
     *
     * <p>This is the cure for the ANTIPODAL stall: with the goal opposite the
     * FK chain, every Jacobian column is perpendicular to the error (a
     * first-order saddle) and the iterations have no gradient to follow. Goals
     * in the FRONT half-space are deliberately left alone: descent handles
     * them, and a full re-aim would trample root stiffness and manufacture the
     * straight-chain degeneracy. Skipped when the root has locked or limited
     * axes — a rigid re-aim would trample what those protect.
     */
    public static void alignToGoal(IKTree tree, int e)
    {
        int rootIndex = tree.rootOf(e);
        IKJoint root = tree.joints[rootIndex];

        for (int c = 0; c < 3; c++)
        {
            if (root.locked[c] || root.limited[c] || root.weight(c) <= 0F)
            {
                return;
            }
        }

        IKTree.Effector effector = tree.effectors[e];
        Vector3f effectorDir = new Vector3f(effector.position).sub(root.position);
        Vector3f goalDir = new Vector3f(effector.goal).sub(root.position);

        if (effectorDir.lengthSquared() < EPS * EPS || goalDir.lengthSquared() < EPS * EPS)
        {
            return;
        }

        effectorDir.normalize();
        goalDir.normalize();

        float angle = effectorDir.angle(goalDir);
        float excess = angle - (float) (Math.PI / 2.0);

        if (excess <= 0F)
        {
            return;
        }

        Vector3f axis = new Vector3f(effectorDir).cross(goalDir);

        if (axis.lengthSquared() < EPS * EPS)
        {
            /* Exact antipode: the turn side is genuinely ambiguous — pick a
             * deterministic perpendicular (the pole snaps the plane after). */
            axis = stablePerpendicular(effectorDir);
        }
        else
        {
            axis.normalize();
        }

        tree.rotateJointWorld(rootIndex, new Quaternionf().rotationAxis(excess, axis.x, axis.y, axis.z));
    }

    /**
     * Blender's pole: twist the chain about its root→goal line so the bend
     * plane (root → first joint towards the effector) contains the pole
     * target, then roll by the pole angle. Skipped when the geometry defines
     * no plane — no interior joint, a goal on the root, or a bend or pole
     * sitting exactly on the twist line (the straight-chain degeneracy).
     * On a merged tree the twist turns the whole branch under the chain's
     * root, other chains included — which is why it is pre-only there.
     */
    public static void applyPole(IKTree tree, Pole pole)
    {
        int e = effectorUnder(tree, pole.rootJoint());

        if (e < 0)
        {
            return;
        }

        int elbowIndex = childTowards(tree, pole.rootJoint(), tree.effectors[e].joint);

        if (elbowIndex < 0)
        {
            return;
        }

        IKJoint root = tree.joints[pole.rootJoint()];
        Vector3f goal = tree.effectors[e].goal;
        Vector3f axis = new Vector3f(goal).sub(root.position);

        if (axis.lengthSquared() < EPS * EPS)
        {
            return;
        }

        axis.normalize();

        Vector3f bend = perpendicular(new Vector3f(tree.joints[elbowIndex].position).sub(root.position), axis);
        Vector3f poleDir = perpendicular(new Vector3f(pole.polePoint()).sub(root.position), axis);

        if (bend == null || poleDir == null)
        {
            return;
        }

        float twist = (float) Math.atan2(new Vector3f(bend).cross(poleDir).dot(axis), bend.dot(poleDir)) + pole.poleAngle();

        if (Math.abs(twist) < EPS)
        {
            return;
        }

        tree.rotateJointWorld(pole.rootJoint(), new Quaternionf().rotationAxis(twist, axis.x, axis.y, axis.z));
    }

    /**
     * Breaks the STRAIGHT-CHAIN degeneracy on a single chain: dead straight
     * with the goal closer than full reach, the Jacobian columns all run
     * perpendicular to the compression the goal asks for and the solve sticks
     * fully extended (Blender locks up the same way; riggers pre-bend the
     * knee). Nudges the first interior joint a fraction of a degree towards
     * the pole side (or a deterministic side) through its most-aligned FREE
     * channel — locks, limits and stiffness are respected. A goal at or past
     * full reach leaves the chain straight, as it should be.
     */
    public static void breakExtension(IKTree tree, int e, Pole pole)
    {
        IKTree.Effector effector = tree.effectors[e];
        int rootIndex = tree.rootOf(e);
        Vector3f root = tree.joints[rootIndex].position;
        float reach = tree.reach(e);
        float goalDistance = root.distance(effector.goal);

        if (reach < EPS || goalDistance >= reach * 0.999F)
        {
            return;
        }

        /* Straightness: the effector of a straight chain sits a full arc length out. */
        if (root.distance(effector.position) < reach * 0.999F)
        {
            return;
        }

        int elbowIndex = childTowards(tree, rootIndex, effector.joint);

        if (elbowIndex < 0)
        {
            return;
        }

        Vector3f goalDir = new Vector3f(effector.goal).sub(root);

        if (goalDir.lengthSquared() < EPS * EPS)
        {
            return;
        }

        goalDir.normalize();

        /* The side to fold towards: the pole's, else a deterministic perpendicular. */
        Vector3f side = pole == null || pole.polePoint() == null ? null : perpendicular(new Vector3f(pole.polePoint()).sub(root), goalDir);

        if (side == null)
        {
            side = stablePerpendicular(goalDir);
        }

        /* The world axis whose turn moves the sub-chain towards that side. */
        Vector3f bendAxis = new Vector3f(goalDir).cross(side).normalize();

        /* The elbow's most-aligned movable channel takes the nudge. */
        IKJoint elbow = tree.joints[elbowIndex];
        Vector3f axis = new Vector3f();
        int best = -1;
        float bestScore = 0F;

        for (int c = 0; c < 3; c++)
        {
            if (movableWeight(elbow, c) <= 0F)
            {
                continue;
            }

            float score = tree.channelAxis(elbowIndex, c, axis).dot(bendAxis);

            if (Math.abs(score) > Math.abs(bestScore))
            {
                best = c;
                bestScore = score;
            }
        }

        if (best < 0 || Math.abs(bestScore) < EPS)
        {
            return;
        }

        IKJoint.set(elbow.angles, best, IKJoint.get(elbow.angles, best) + Math.signum(bestScore) * 0.02F);
        elbow.clampLimits();
        tree.forward();
    }

    /* ------------------------------------------------------------------ */

    private static Pole firstPole(Pole[] poles)
    {
        return poles == null || poles.length == 0 ? null : poles[0];
    }

    /** The first effector whose branch passes through the given joint. */
    private static int effectorUnder(IKTree tree, int joint)
    {
        for (int e = 0; e < tree.effectors.length; e++)
        {
            if (tree.moves(e, joint))
            {
                return e;
            }
        }

        return -1;
    }

    /** The next joint after {@code from} on the path down to {@code descendant}; -1 when none. */
    private static int childTowards(IKTree tree, int from, int descendant)
    {
        int j = descendant;
        int previous = -1;

        while (j >= 0 && j != from)
        {
            previous = j;
            j = tree.parentIndex[j];
        }

        return j == from ? previous : -1;
    }

    private static float largestReach(IKTree tree)
    {
        float largest = 0F;

        for (int e = 0; e < tree.effectors.length; e++)
        {
            largest = Math.max(largest, tree.reach(e));
        }

        return largest;
    }

    /** The largest plain effector-to-goal distance — the user-facing "did it reach". */
    private static float worstError(IKTree tree)
    {
        float worst = 0F;

        for (IKTree.Effector effector : tree.effectors)
        {
            worst = Math.max(worst, effector.position.distance(effector.goal));
        }

        return worst;
    }

    /** The weighted least-squares error the step must not increase. */
    private static float combinedError(IKTree tree)
    {
        float sum = 0F;

        for (IKTree.Effector effector : tree.effectors)
        {
            float distance = effector.position.distance(effector.goal) * effector.weight;

            sum += distance * distance;
        }

        return (float) Math.sqrt(sum);
    }

    /** How many channels can move at all — zero means there is nothing to solve with. */
    private static int countColumns(IKTree tree)
    {
        int columns = 0;

        for (IKJoint joint : tree.joints)
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

    /** How much this channel may move at all: 0 when locked, weightless, or pinned by min == max limits. */
    private static float movableWeight(IKJoint joint, int axis)
    {
        if (joint.locked[axis] || (joint.limited[axis] && joint.limitMin[axis] >= joint.limitMax[axis]))
        {
            return 0F;
        }

        return joint.weight(axis);
    }

    /** One damped-least-squares iteration; false when the step stalls. */
    private static boolean step(IKTree tree, float lambdaSq)
    {
        IKJoint[] joints = tree.joints;
        int n = joints.length;
        int k = tree.effectors.length;
        int rows = k * 3;
        float errorBefore = combinedError(tree);

        /* Row-weighted error vector b (per effector: weight · (goal − position)). */
        float[] b = new float[rows];

        for (int e = 0; e < k; e++)
        {
            IKTree.Effector effector = tree.effectors[e];

            b[e * 3] = effector.weight * (effector.goal.x - effector.position.x);
            b[e * 3 + 1] = effector.weight * (effector.goal.y - effector.position.y);
            b[e * 3 + 2] = effector.weight * (effector.goal.z - effector.position.z);
        }

        /* Weighted Jacobian columns: per channel a 3k vector — for every effector
         * the channel moves, colWeight · rowWeight · (axis × (effector − pivot)).
         * A channel pinned by its limits (min == max) never enters. */
        float[] columns = new float[n * 3 * rows];
        boolean[] frozen = new boolean[n * 3];
        Vector3f axis = new Vector3f();
        Vector3f lever = new Vector3f();

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

                tree.channelAxis(i, c, axis);

                int at = (i * 3 + c) * rows;

                for (int e = 0; e < k; e++)
                {
                    if (!tree.moves(e, i))
                    {
                        continue;
                    }

                    IKTree.Effector effector = tree.effectors[e];

                    /* cross(v, dest) leaves the axis itself untouched. */
                    lever.set(effector.position).sub(joint.position);
                    axis.cross(lever, lever).mul(w * effector.weight);

                    columns[at + e * 3] = lever.x;
                    columns[at + e * 3 + 1] = lever.y;
                    columns[at + e * 3 + 2] = lever.z;
                }
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
        float[] deltas = solveDeltas(tree, columns, frozen, b, lambdaSq);

        if (deltas == null)
        {
            return false;
        }

        float largest = applyDeltas(tree, deltas, 1F);

        tree.forward();

        if (combinedError(tree) <= errorBefore)
        {
            return largest > STALL_STEP;
        }

        /* Phase 2 (active set): when the limit clamp CUT part of that step, what
         * was applied is no longer a descent direction — freeze the cut channels
         * and re-solve over what can actually move. */
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

        restoreAngles(tree, saved);

        if (cut)
        {
            deltas = solveDeltas(tree, columns, frozen, b, lambdaSq);

            if (deltas != null)
            {
                largest = applyDeltas(tree, deltas, 1F);
                tree.forward();

                if (combinedError(tree) <= errorBefore)
                {
                    return largest > STALL_STEP;
                }

                restoreAngles(tree, saved);
            }
        }

        /* Phase 3 (backtracking): the step overshot — retry at halved scales; if
         * even the smallest scale is worse, this is the local best — stop. Error
         * is monotonically non-increasing, frames stay steady. */
        if (deltas != null)
        {
            float scale = 0.5F;

            for (int attempt = 0; attempt < 6; attempt++)
            {
                largest = applyDeltas(tree, deltas, scale);
                tree.forward();

                if (combinedError(tree) <= errorBefore)
                {
                    return largest > STALL_STEP;
                }

                restoreAngles(tree, saved);
                scale *= 0.5F;
            }
        }

        tree.forward();

        return false;
    }

    /**
     * Solves the damped normal equations over the unfrozen columns and returns
     * the per-channel deltas {@code W·Jᵀ·y} (the second W of {@code W²·Ĵᵀ·y} —
     * the first is inside the stored columns), the whole vector scaled down
     * when its largest entry exceeds {@link #MAX_STEP} (direction preserved);
     * {@code null} when the system is singular.
     */
    private static float[] solveDeltas(IKTree tree, float[] columns, boolean[] frozen, float[] b, float lambdaSq)
    {
        IKJoint[] joints = tree.joints;
        int n = joints.length;
        int rows = b.length;

        /* A = Σ c·cᵀ + λ²I, symmetric 3k×3k. */
        float[] a = new float[rows * rows];

        for (int r = 0; r < rows; r++)
        {
            a[r * rows + r] = lambdaSq;
        }

        for (int at = 0; at < n * 3; at++)
        {
            if (frozen[at])
            {
                continue;
            }

            int base = at * rows;

            for (int r = 0; r < rows; r++)
            {
                float cr = columns[base + r];

                if (cr == 0F)
                {
                    continue;
                }

                for (int s = r; s < rows; s++)
                {
                    a[r * rows + s] += cr * columns[base + s];
                }
            }
        }

        float[] y = solveCholesky(a, b, rows);

        if (y == null)
        {
            return null;
        }

        float[] deltas = new float[n * 3];
        float largest = 0F;

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
                float delta = 0F;
                int base = at * rows;

                for (int r = 0; r < rows; r++)
                {
                    delta += columns[base + r] * y[r];
                }

                deltas[at] = w * delta;
                largest = Math.max(largest, Math.abs(deltas[at]));
            }
        }

        /* Cap by scaling the WHOLE vector, never per channel: a per-channel clamp
         * saturates every delta at a large |y| and the step degenerates into a
         * vector of signs — no longer a descent direction. */
        if (largest > MAX_STEP)
        {
            float scale = MAX_STEP / largest;

            for (int at = 0; at < n * 3; at++)
            {
                deltas[at] *= scale;
            }
        }

        return deltas;
    }

    /**
     * Solves the symmetric positive-definite system A·y = b (upper triangle of
     * A filled) by Cholesky decomposition; {@code null} when not positive
     * definite (numerically singular). The damping term guarantees positive
     * definiteness in practice.
     */
    private static float[] solveCholesky(float[] a, float[] b, int size)
    {
        /* Decompose A = L·Lᵀ in place (L in the lower triangle). */
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                float sum = a[Math.min(i, j) * size + Math.max(i, j)];

                for (int m = 0; m < j; m++)
                {
                    sum -= a[i * size + m] * a[j * size + m];
                }

                if (i == j)
                {
                    if (sum <= 0F)
                    {
                        return null;
                    }

                    a[i * size + i] = (float) Math.sqrt(sum);
                }
                else
                {
                    a[i * size + j] = sum / a[j * size + j];
                }
            }
        }

        /* Forward substitution L·z = b, then back substitution Lᵀ·y = z. */
        float[] y = new float[size];

        for (int i = 0; i < size; i++)
        {
            float sum = b[i];

            for (int m = 0; m < i; m++)
            {
                sum -= a[i * size + m] * y[m];
            }

            y[i] = sum / a[i * size + i];
        }

        for (int i = size - 1; i >= 0; i--)
        {
            float sum = y[i];

            for (int m = i + 1; m < size; m++)
            {
                sum -= a[m * size + i] * y[m];
            }

            y[i] = sum / a[i * size + i];
        }

        return y;
    }

    /** Applies {@code scale}-sized deltas onto the current angles; returns the largest applied delta. */
    private static float applyDeltas(IKTree tree, float[] deltas, float scale)
    {
        float largest = 0F;

        for (int i = 0; i < tree.joints.length; i++)
        {
            IKJoint joint = tree.joints[i];

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

    private static void restoreAngles(IKTree tree, float[] saved)
    {
        for (int i = 0; i < tree.joints.length; i++)
        {
            IKJoint joint = tree.joints[i];

            for (int c = 0; c < 3; c++)
            {
                IKJoint.set(joint.angles, c, saved[i * 3 + c]);
            }
        }
    }

    /** {@code v} minus its component along unit {@code axis}, normalized; {@code null} when degenerate. */
    private static Vector3f perpendicular(Vector3f v, Vector3f axis)
    {
        v.fma(-v.dot(axis), axis);

        return v.lengthSquared() < EPS * EPS ? null : v.normalize();
    }

    /** A deterministic unit perpendicular to {@code dir}: cross with world Z, falling back to world Y. */
    private static Vector3f stablePerpendicular(Vector3f dir)
    {
        Vector3f perp = new Vector3f(dir).cross(0F, 0F, 1F);

        if (perp.lengthSquared() < EPS * EPS)
        {
            perp.set(dir).cross(0F, 1F, 0F);
        }

        return perp.normalize();
    }
}
