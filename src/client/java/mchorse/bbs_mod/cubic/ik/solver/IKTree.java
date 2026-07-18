package mchorse.bbs_mod.cubic.ik.solver;

import mchorse.bbs_mod.utils.joml.Matrices;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.Arrays;

/**
 * An IK tree in world (model) space: the union of one or more OVERLAPPING
 * chains solved together, Blender-style — joints (bones with channel angles),
 * each hanging under its nearest captured ancestor, and one positional
 * effector per merged chain. A single chain is simply the linear,
 * one-effector case.
 *
 * <p>The forward model is the RIGID re-rotation of the captured start frames,
 * generalized from the chain to a forest: a joint's parent frame is its
 * captured parent frame carried by how far its nearest captured ANCESTOR's
 * world rotation moved (bones between them, if any, hold constant angles
 * during the solve, so they ride inside the captured relation):
 *
 * <pre>
 * ΔW(a)        = worldRot[a] · startWorldRot[a]⁻¹
 * parentRot[i] = ΔW(anc(i)) · startParentRot[i]
 * worldRot[i]  = parentRot[i] · Rz·Ry·Rx(angles[i])
 * p[i]         = p[anc(i)] + ΔW(anc(i)) · (startPos[i] − startPos[anc(i)]) · (1 + stretch[anc(i)])
 * </pre>
 *
 * A joint with no captured ancestor is a ROOT of the forest: its pivot and
 * parent frame are frozen for the solve. Effectors ride their chain's last
 * directed joint the same rigid way. Exact for any ancestor/link scale (scale
 * lives inside the captured segments — see the chain-era derivation, verified
 * against the real renderer to float precision).
 *
 * <p>{@link IKJoint#stretch} scales a joint's OUTGOING segments — the ones
 * that carry its children and, for the chain's last directed bone, its
 * effector — because that is the segment the bone's geometry occupies. A bone
 * that stretches therefore pushes everything below it further out, exactly as
 * lengthening a Blender bone does, while its own pivot stays put.
 */
public final class IKTree
{
    /** Joints in parents-first order (a joint's ancestor sits earlier in the array). */
    public final IKJoint[] joints;

    /** Index of each joint's nearest captured ancestor; -1 = a root of the forest. */
    public final int[] parentIndex;

    /** Captured parent frame of each joint (its MODEL parent's world rotation at capture). */
    public final Quaternionf[] startParentRotation;

    /** One per merged chain: where its effector is and what it wants. */
    public final Effector[] effectors;

    /**
     * A chain's positional goal inside the tree: the effector point rides
     * {@code joint} (the chain's last directed bone); {@code weight} is the
     * chain's solve priority when goals conflict — every Jacobian row of this
     * effector is scaled by it, so a lighter goal yields to a heavier one.
     *
     * <p>With {@link #orientGoal} set, the effector ALSO asks for a world
     * orientation of its joint — three more Jacobian rows (a joint axis turns
     * the whole subtree's orientation by itself, so the rows are the bare
     * channel axes). That is how "tip follows target" becomes the chain's
     * problem: the goal is the orientation at which the tip bone, keeping its
     * natural FK local pose, would already face the controller — the chain
     * turns the forearm so the exact post-solve tip snap has almost nothing
     * left to do, instead of the wrist absorbing the whole turn.
     * {@code orientWeight} converts radians of orientation error into the
     * solve's length units (rows are scaled by weight · orientWeight).
     */
    public static final class Effector
    {
        /** Index of the chain's last directed joint in {@link #joints}. */
        public final int joint;

        /** World effector position at capture. */
        public final Vector3f startPosition = new Vector3f();

        /** World effector position at the current angles, refreshed by {@link #forward()}. */
        public final Vector3f position = new Vector3f();

        /** The goal this effector reaches for, world space. */
        public final Vector3f goal = new Vector3f();

        /** Solve priority, 0..1. */
        public float weight = 1F;

        /** Desired world orientation of {@link #joint}; {@code null} = position only. */
        public Quaternionf orientGoal;

        /** Length units one radian of orientation error is worth; scales the orientation rows. */
        public float orientWeight = 1F;

        public Effector(int joint)
        {
            this.joint = joint;
        }
    }

    /** [effector][joint]: whether the joint's turn moves that effector (tree ancestry). */
    private final boolean[][] moves;

    /**
     * [effector][joint]: the next joint DOWN the path towards that effector, or
     * {@code -1} for the effector's own joint (whose outgoing segment ends at
     * the effector point itself). Only meaningful where {@link #moves} is set.
     * This is what tells a stretching joint which of its outgoing segments
     * carries a given effector.
     */
    private final int[][] next;

    public IKTree(int jointCount, int effectorCount)
    {
        this.joints = new IKJoint[jointCount];
        this.parentIndex = new int[jointCount];
        this.startParentRotation = new Quaternionf[jointCount];
        this.effectors = new Effector[effectorCount];
        this.moves = new boolean[effectorCount][jointCount];
        this.next = new int[effectorCount][jointCount];

        for (int i = 0; i < jointCount; i++)
        {
            this.joints[i] = new IKJoint();
            this.parentIndex[i] = -1;
            this.startParentRotation[i] = new Quaternionf();
        }

        for (int e = 0; e < effectorCount; e++)
        {
            Arrays.fill(this.next[e], -1);
        }
    }

    /** Sets effector {@code e} to ride joint {@code joint}; call after parentIndex is filled. */
    public Effector effector(int e, int joint)
    {
        Effector effector = new Effector(joint);

        this.effectors[e] = effector;

        for (int j = joint; j >= 0; j = this.parentIndex[j])
        {
            this.moves[e][j] = true;

            int parent = this.parentIndex[j];

            if (parent >= 0)
            {
                this.next[e][parent] = j;
            }
        }

        return effector;
    }

    /** Whether turning joint {@code joint} moves effector {@code e}. */
    public boolean moves(int e, int joint)
    {
        return this.moves[e][joint];
    }

    /** Total captured arc length from the given chain root joint down to effector {@code e} — its reach. */
    public float reach(int e)
    {
        Effector effector = this.effectors[e];
        float total = effector.startPosition.distance(this.joints[effector.joint].startPosition);
        int j = effector.joint;

        while (this.parentIndex[j] >= 0)
        {
            int parent = this.parentIndex[j];

            total += this.joints[j].startPosition.distance(this.joints[parent].startPosition);
            j = parent;
        }

        return total;
    }

    /**
     * The effector's reach with every bone stretched to its limit — how far the
     * chain can get at all. The soft-goal preprocessor works against THIS, not
     * the natural {@link #reach(int)}: easing the goal onto the natural reach
     * sphere would hide the very shortfall stretching exists to cover, and a
     * stretchable chain would never learn that its target sits further out.
     */
    public float stretchedReach(int e)
    {
        Effector effector = this.effectors[e];
        IKJoint last = this.joints[effector.joint];
        float total = effector.startPosition.distance(last.startPosition) * last.maxStretchScale();
        int j = effector.joint;

        while (this.parentIndex[j] >= 0)
        {
            int parent = this.parentIndex[j];
            IKJoint owner = this.joints[parent];

            total += this.joints[j].startPosition.distance(owner.startPosition) * owner.maxStretchScale();
            j = parent;
        }

        return total;
    }

    /** The forest-root joint the effector's branch hangs from. */
    public int rootOf(int e)
    {
        int j = this.effectors[e].joint;

        while (this.parentIndex[j] >= 0)
        {
            j = this.parentIndex[j];
        }

        return j;
    }

    /**
     * Re-poses the whole tree from the current {@link IKJoint#angles}: refreshes
     * every joint's {@code position}/{@code parentRotation}/{@code worldRotation}
     * and every effector's {@code position}. See the class doc for the model.
     */
    public void forward()
    {
        for (int i = 0; i < this.joints.length; i++)
        {
            IKJoint joint = this.joints[i];
            int parent = this.parentIndex[i];

            if (parent < 0)
            {
                joint.position.set(joint.startPosition);
                joint.parentRotation.set(this.startParentRotation[i]);
            }
            else
            {
                IKJoint ancestor = this.joints[parent];
                Quaternionf delta = delta(ancestor);

                joint.parentRotation.set(delta).mul(this.startParentRotation[i]);

                Vector3f segment = new Vector3f(joint.startPosition).sub(ancestor.startPosition);

                delta.transform(segment).mul(1F + ancestor.stretch);
                joint.position.set(ancestor.position).add(segment);
            }

            joint.worldRotation.set(joint.parentRotation).mul(Matrices.toLocalRotationZYXRadians(joint.angles));
        }

        for (Effector effector : this.effectors)
        {
            IKJoint last = this.joints[effector.joint];
            Vector3f segment = new Vector3f(effector.startPosition).sub(last.startPosition);

            delta(last).transform(segment).mul(1F + last.stretch);
            effector.position.set(last.position).add(segment);
        }
    }

    /** How far the joint's world rotation moved from capture: {@code worldRot · startWorldRot⁻¹}. */
    private static Quaternionf delta(IKJoint joint)
    {
        return new Quaternionf(joint.worldRotation).mul(new Quaternionf(joint.startWorldRotation).conjugate());
    }

    /**
     * The world axis the given rotation channel currently turns about — the
     * Jacobian column direction. ZYX stack: channel Z turns about the parent
     * frame's Z, channel Y about the Z-rotated Y, channel X about the fully
     * rotated X (Rx commutes with its own axis).
     */
    public Vector3f channelAxis(int jointIndex, int axis, Vector3f dest)
    {
        IKJoint joint = this.joints[jointIndex];

        if (axis == 0)
        {
            return joint.worldRotation.transform(dest.set(1F, 0F, 0F));
        }

        if (axis == 1)
        {
            return new Quaternionf(joint.parentRotation).rotateZ(joint.angles.z).transform(dest.set(0F, 1F, 0F));
        }

        return joint.parentRotation.transform(dest.set(0F, 0F, 1F));
    }

    /**
     * The Jacobian column of joint {@code jointIndex}'s STRETCH variable for
     * effector {@code e}: the world vector of the outgoing segment that carries
     * that effector, at its NATURAL (unstretched) length.
     *
     * <p>Exact, not an approximation: the forward model places everything below
     * the joint at {@code p + (1 + stretch) · segment}, so the derivative of any
     * descendant position — the effector included — with respect to
     * {@code stretch} is that very segment. Dividing the current segment by
     * {@code 1 + stretch} recovers it. Orientation is untouched by a stretch, so
     * its orientation rows are zero.
     */
    public Vector3f stretchAxis(int e, int jointIndex, Vector3f dest)
    {
        IKJoint joint = this.joints[jointIndex];
        int child = this.next[e][jointIndex];
        Vector3f target = child < 0 ? this.effectors[e].position : this.joints[child].position;
        float scale = 1F + joint.stretch;

        dest.set(target).sub(joint.position);

        return scale > 1.0e-6f ? dest.div(scale) : dest;
    }

    /**
     * Rigidly rotates joint {@code index} — and with it everything below — by the
     * world rotation {@code delta} about its own pivot, by folding the delta into
     * the joint's channel angles (compatible-euler, anchored to the current
     * angles so continuity and winding survive), clamping them back into the
     * joint's limits, and re-running {@link #forward()}. The pole constraint's
     * root twist, the goal pre-alignment and the extension nudge go through
     * here, so every mutation of the tree flows through the one variable — the
     * angles.
     */
    public void rotateJointWorld(int index, Quaternionf delta)
    {
        IKJoint joint = this.joints[index];
        Quaternionf world = new Quaternionf(delta).mul(joint.worldRotation);
        Quaternionf local = new Quaternionf(joint.parentRotation).conjugate().mul(world);

        Matrices.toCompatibleEulerZYXRadians(local, joint.angles, joint.angles);
        joint.clampLimits();
        this.forward();
    }
}
