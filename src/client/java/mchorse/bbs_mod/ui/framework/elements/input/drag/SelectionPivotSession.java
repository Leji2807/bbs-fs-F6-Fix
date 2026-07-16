package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.utils.joml.Matrices;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.List;

/**
 * A multi-bone transform gesture about a common pivot ({@link PivotMode#MEDIAN}
 * / {@link PivotMode#ACTIVE}): the selection moves as one rigid group, the way
 * Blender transforms a multi-selection. Built once when the edit starts — every
 * per-bone quantity (world origin, channel axes, translate Jacobian, channel
 * snapshot) is measured at grab time — and driven each frame by the active
 * {@link DragStrategy} with its accumulated WORLD delta, always against those
 * fixed captures, never against the previous frame's result.
 *
 * <p>Only the selection's roots (bones without a selected ancestor) receive
 * writes: a rigid group motion leaves a selected descendant's local channels
 * untouched, since its parent already carries it. Rotation lands on each root
 * twice — the turn of the bone about its own origin (composed in its parent
 * frame, read back through the shared compatible-euler unwrap) and the orbit
 * of its origin about the pivot (converted to translate channels through the
 * bone's own Jacobian).
 *
 * <p>The session deliberately bypasses the per-channel fan-out of the delta
 * editors (and with it the mirror-edit partner extension): a common-pivot edit
 * is a geometric group operation, while mirror/invert remain channel workflows
 * of {@link PivotMode#INDIVIDUAL}.
 */
public class SelectionPivotSession
{
    private final List<BoneCapture> bones;
    private final Vector3f pivot;

    /** World offset folded in by incremental side-moves (the depth wheel), which
     *  re-anchor the strategy's own accumulation; added under every translation. */
    private final Vector3f foldedWorld = new Vector3f();

    /** Undo/keyframe hooks of the host editor, wrapped around every write batch. */
    private final Runnable preWrite;
    private final Runnable postWrite;

    /** Re-reads the editor's value fields from the primary transform after a batch. */
    private final Runnable refresh;

    public SelectionPivotSession(List<BoneCapture> bones, Vector3f pivot, Runnable preWrite, Runnable postWrite, Runnable refresh)
    {
        this.bones = bones;
        this.pivot = pivot;
        this.preWrite = preWrite;
        this.postWrite = postWrite;
        this.refresh = refresh;
    }

    /**
     * Everything the session measured about one selected bone at grab time.
     * Non-drivers (bones with a selected ancestor) carry only the snapshot,
     * for {@link #restore()}; they ride their parent during the gesture.
     */
    public static class BoneCapture
    {
        /** The bone's live (written) transform. */
        public final Transform transform;

        /** Channel snapshot at grab; the fixed base every frame composes against. */
        public final Transform cache;

        /** World position of the bone's origin at grab. */
        public final Vector3f worldOrigin;

        /** World direction → bone parent frame, or {@code null} when degenerate (rotation skipped). */
        public final Matrix3f parentInverse;

        /** World displacement → translate channel units, or {@code null} when degenerate (orbit skipped). */
        public final Matrix3f jacobianInverse;

        /** Whether this bone receives writes (no selected ancestor). */
        public final boolean driver;

        public BoneCapture(Transform transform, Vector3f worldOrigin, Matrix3f parentInverse, Matrix3f jacobianInverse, boolean driver)
        {
            this.transform = transform;
            this.cache = transform.copy();
            this.worldOrigin = worldOrigin;
            this.parentInverse = parentInverse;
            this.jacobianInverse = jacobianInverse;
            this.driver = driver;
        }
    }

    /**
     * Apply the gesture's total world rotation about the pivot: each driver
     * turns about its own origin by {@code deltaWorld} (conjugated into its
     * parent frame, composed onto the cached angles) and its origin orbits the
     * pivot (converted to translate channels through the bone's Jacobian).
     *
     * @param windingReference how the euler readback of each bone is anchored —
     *        {@code true} reads against the LIVE channels (strict continuity, so
     *        the ring's multi-turn winding survives), {@code false} against the
     *        GRAB snapshot (pure function of the gesture; the free rotations use
     *        it so a near-pole passage self-recovers instead of stranding X/Z
     *        at ±180 — see {@link RotationDragMath#writeCompatibleEuler}).
     */
    public void applyRotation(Matrix3f deltaWorld, boolean windingReference)
    {
        this.preWrite.run();

        Vector3f euler = new Vector3f();
        Vector3f relative = new Vector3f();

        for (BoneCapture bone : this.bones)
        {
            if (!bone.driver)
            {
                continue;
            }

            if (bone.parentInverse != null)
            {
                /* The world turn expressed in the bone's parent frame: P⁻¹·ΔR·P. */
                Matrix3f parentDelta = new Matrix3f(bone.parentInverse)
                    .mul(deltaWorld)
                    .mul(new Matrix3f(bone.parentInverse).invert());

                /* Mode fork mirrors RotationDragMath.applyLocalDelta: a quaternion
                 * bone takes the delta as a quaternion product onto its cached
                 * quat (its euler channels are stale — composing/writing them
                 * would leave the bone orbiting the pivot without turning);
                 * an euler bone keeps the compatible-euler channel write. */
                if (bone.transform.rotationMode == Transform.RotationMode.QUATERNION)
                {
                    bone.transform.quat.set(new Quaternionf().setFromNormalized(parentDelta).mul(bone.cache.quat));
                }
                else
                {
                    Matrix3f composed = parentDelta.mul(RotationDragMath.eulerZYX(bone.cache.rotate));
                    Vector3f reference = windingReference ? bone.transform.rotate : bone.cache.rotate;

                    Matrices.toCompatibleEulerZYXRadians(composed, reference, euler);

                    bone.transform.rotate.set(euler);
                }
            }

            relative.set(bone.worldOrigin).sub(this.pivot);
            deltaWorld.transform(relative);
            relative.add(this.pivot).sub(bone.worldOrigin);

            this.applyWorldShift(bone, relative);
        }

        this.postWrite.run();
        this.refresh.run();
    }

    /** Apply the gesture's total world displacement to every driver, through each bone's own Jacobian. */
    public void applyTranslation(Vector3f deltaWorld)
    {
        this.preWrite.run();

        Vector3f total = new Vector3f(this.foldedWorld).add(deltaWorld);

        for (BoneCapture bone : this.bones)
        {
            if (bone.driver)
            {
                this.applyWorldShift(bone, total);
            }
        }

        this.postWrite.run();
        this.refresh.run();
    }

    /** Fold an incremental world step (the depth wheel) into the session and re-apply. */
    public void foldTranslation(Vector3f worldStep)
    {
        this.foldedWorld.add(worldStep);
        this.applyTranslation(new Vector3f());
    }

    /** {@code translate = cache + J⁻¹ · world shift}; skipped for bones with a degenerate Jacobian. */
    private void applyWorldShift(BoneCapture bone, Vector3f worldShift)
    {
        if (bone.jacobianInverse == null)
        {
            return;
        }

        Vector3f channels = bone.jacobianInverse.transform(worldShift, new Vector3f());

        bone.transform.translate.set(bone.cache.translate).add(channels);
    }

    /** Rewind every captured bone (drivers and riders alike) to its grab snapshot. */
    public void restore()
    {
        this.preWrite.run();

        for (BoneCapture bone : this.bones)
        {
            bone.transform.copy(bone.cache);
        }

        this.postWrite.run();
        this.refresh.run();
    }
}
