package mchorse.bbs_mod.cubic.ik.solver;

import mchorse.bbs_mod.utils.joml.Matrices;
import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * An IK chain in world (model) space: the directed joints root to tip plus the
 * effector point beyond the last joint, with the forward model that re-poses
 * the captured frames from the current channel angles.
 *
 * <p>The forward model is a RIGID re-rotation of the captured start frames —
 * positions are never rebuilt from local bone geometry. Each bone's world
 * rotation advances by the quaternion chain {@code parent · Rz·Ry·Rx(angles)},
 * and each captured world segment is carried by how far its bone's world
 * rotation moved from capture:
 *
 * <pre>
 * p[i+1] = p[i] + (worldRot[i] · startWorldRot[i]⁻¹) · (startPos[i+1] − startPos[i])
 * </pre>
 *
 * This is exactly the renderer's response to changing the rotation channels
 * (verified against {@code CubicRenderer.collectPivotFrames} to float
 * precision): any ancestor or per-link scale lives baked inside the captured
 * segments, where rotations can't disturb it, so the model stays exact for
 * scaled rigs without ever knowing about scale. Ancestors outside the chain
 * are frozen for the duration of a solve — their frame enters only through
 * {@link #rootParentRotation} and the captured root position.
 */
public final class IKChain
{
    /** Directed bones, root first. The effector hangs off the last one. */
    public final IKJoint[] joints;

    /** World effector position at capture (tip pivot, or the tail marker). */
    public final Vector3f startEffector = new Vector3f();

    /** World effector position at the current angles, refreshed by {@link #forward()}. */
    public final Vector3f effector = new Vector3f();

    /** The frozen world rotation of the chain root's parent frame. */
    public final Quaternionf rootParentRotation = new Quaternionf();

    public IKChain(int jointCount)
    {
        this.joints = new IKJoint[jointCount];

        for (int i = 0; i < jointCount; i++)
        {
            this.joints[i] = new IKJoint();
        }
    }

    /** Total captured length of the chain, root pivot to effector — the reach. */
    public float totalLength()
    {
        float total = 0F;
        Vector3f previous = this.joints[0].startPosition;

        for (int i = 1; i < this.joints.length; i++)
        {
            total += previous.distance(this.joints[i].startPosition);
            previous = this.joints[i].startPosition;
        }

        return total + previous.distance(this.startEffector);
    }

    /**
     * Re-poses the chain from the current {@link IKJoint#angles}: refreshes every
     * joint's {@code position}/{@code parentRotation}/{@code worldRotation} and
     * the {@link #effector}. See the class doc for the model.
     */
    public void forward()
    {
        Quaternionf parent = new Quaternionf(this.rootParentRotation);
        Vector3f position = new Vector3f(this.joints[0].startPosition);

        for (int i = 0; i < this.joints.length; i++)
        {
            IKJoint joint = this.joints[i];

            joint.position.set(position);
            joint.parentRotation.set(parent);
            joint.worldRotation.set(parent).mul(Matrices.toLocalRotationZYXRadians(joint.angles));

            Vector3f nextStart = i + 1 < this.joints.length ? this.joints[i + 1].startPosition : this.startEffector;
            Vector3f segment = new Vector3f(nextStart).sub(joint.startPosition);
            Quaternionf delta = new Quaternionf(joint.worldRotation).mul(new Quaternionf(joint.startWorldRotation).conjugate());

            delta.transform(segment);
            position.add(segment);
            parent.set(joint.worldRotation);
        }

        this.effector.set(position);
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
     * Rigidly rotates joint {@code index} — and with it everything below — by the
     * world rotation {@code delta} about its own pivot, by folding the delta into
     * the joint's channel angles (compatible-euler, anchored to the current
     * angles so continuity and winding survive) and re-running {@link #forward()}.
     * The angles are clamped back into the joint's limits, so the solver's
     * invariant — angles always within limits — holds through every mutation.
     * The pole constraint's root twist, the goal pre-alignment and any seed nudge
     * go through here, so every mutation of the chain flows through the one
     * variable — the angles.
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
