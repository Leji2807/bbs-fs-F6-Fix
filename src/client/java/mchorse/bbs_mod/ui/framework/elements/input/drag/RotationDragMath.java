package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.ui.utils.GizmoDrag;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.joml.Matrices;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector2f;
import org.joml.Vector3f;

/**
 * The euler glue shared by every rotation drag: compose the turn as a
 * matrix, read ZYX euler angles back out, and write them on the branch
 * continuous with the live pose so a value never jumps mid-gesture. Used to be
 * copy-pasted across the view/trackball/arcball drags and the typed-angle
 * input.
 */
public final class RotationDragMath
{
    private RotationDragMath()
    {}

    /**
     * Cursor angle (radians) around a projected centre, in the viewport pixel
     * convention (Y down) so it lines up with {@link GizmoDrag#projectToScreen}.
     */
    public static float screenAngle(Vector2f center, int mouseX, int mouseY)
    {
        return (float) Math.atan2(mouseY - center.y, mouseX - center.x);
    }

    /**
     * Unwrap a frame-to-frame angle step across the ±180° seam so a small
     * motion there registers as a small step instead of a near-full turn the
     * other way.
     */
    public static float wrapSeamRad(float delta)
    {
        if (delta > MathUtils.PI) return delta - MathUtils.PI * 2F;
        if (delta < -MathUtils.PI) return delta + MathUtils.PI * 2F;

        return delta;
    }

    /** {@code Rz·Ry·Rx} rotation from euler radians — the renderer's composition order. */
    public static Matrix3f eulerZYX(Vector3f radians)
    {
        return new Matrix3f().rotationZ(radians.z).rotateY(radians.y).rotateX(radians.x);
    }

    /**
     * The grab pose's ZYX euler, mode-aware: in QUATERNION mode the euler
     * channels are stale, so this is the cache quaternion's ZYX equivalent —
     * the exact angles {@link GizmoDrag#computeRotateAxes} perturbs, so the
     * source and the measured axes stay consistent.
     */
    public static Vector3f cacheSourceEuler(DragContext ctx)
    {
        return sourceEuler(ctx.cache());
    }

    /**
     * A transform's effective ZYX euler as the axis-measurement source, mode-aware:
     * in QUATERNION mode the euler channels are stale, so this decomposes the live
     * quaternion — the exact angles {@link GizmoDrag#computeRotateAxes} perturbs.
     * Feeding the stale channels to {@link #computeParentInverse} instead would
     * evaluate {@link #eulerAxes} at a different orientation than the measured
     * {@code rotateAxes} and recover a wrong parent frame.
     */
    public static Vector3f sourceEuler(Transform transform)
    {
        return transform.rotationMode == Transform.RotationMode.QUATERNION
            ? Matrices.toEulerZYXRadians(transform.quat, new Vector3f())
            : new Vector3f(transform.rotate);
    }

    /**
     * The bone's parent-frame inverse for a world/view-axis turn, built from the
     * MEASURED rotate axes ({@link GizmoDrag#computeRotateAxes}). Those fold in
     * the parent chain AND any model flip (the cubic {@code Ry(180)} the renderer
     * post-multiplies AFTER the bone's own rotation), so a world axis maps into
     * the correct local-delta frame. The raw bone orientation does NOT: since the
     * flip is post-multiplied, dividing it out leaves an extra conjugation that
     * turns X/Z backwards on cubic bones (see [[cubic-ry180-flip-gotcha]] — this
     * regressed trackball/view/arcball there). Returns {@code null} near a gimbal
     * pole, where the euler axes degenerate; the caller then skips the gesture.
     */
    public static Matrix3f parentInverse(DragContext ctx, GizmoDrag drag)
    {
        return computeParentInverse(drag.rotateAxes, cacheSourceEuler(ctx));
    }

    /**
     * Decompose {@code rotation} to ZYX euler and write it into the transform's
     * rotate channel as the representation nearest {@code referenceRadians}
     * (branch and winding), via {@link Matrices#toCompatibleEulerZYXRadians}.
     *
     * <p>The choice of reference is the whole game here — the euler map has a
     * BRANCH POINT at the middle-angle pole (y = ±90°), so a drag path passing
     * near the pole continuously flows onto the flipped branch
     * {@code (x±180, 180−y, z±180)}; no unwrap can prevent that, only the anchor
     * decides where the values settle:
     *
     * <ul>
     * <li><b>The GRAB euler</b> (the same base the delta composes onto) makes the
     * written channels a pure function of the gesture: a near-pole passage shows
     * a brief large-X/Z transient (those orientations genuinely have no
     * small-X/Z representation) but self-recovers to the branch nearest the
     * grab once past — a clean Y-sweep ends as {@code (small, y, small)}, and a
     * pre-existing wound pose (y=720°) keeps its winding since the anchor
     * carries it. Free rotations (trackball/arcball/view) use this.</li>
     * <li><b>The LIVE channels</b> (previous frame's write) follow strict
     * continuity, which accumulates winding across full turns — but a near-pole
     * passage then STRANDS the values on the far branch (X/Z parked at ±180,
     * y folding back) for the rest of the gesture. Only the ring uses this,
     * where multi-turn winding is the feature and the axis is fixed (an
     * axis-aligned corridor crosses poles cleanly; see the ring).</li>
     * </ul>
     */
    public static void writeCompatibleEuler(DragContext ctx, Matrix3f rotation, Vector3f referenceRadians)
    {
        Vector3f euler = Matrices.toCompatibleEulerZYXRadians(rotation, referenceRadians, new Vector3f());

        ctx.writeRotateDeg(MathUtils.toDeg(euler.x), MathUtils.toDeg(euler.y), MathUtils.toDeg(euler.z));
    }

    /**
     * Compose a parent-frame delta rotation onto the grab base and write it per
     * the edited transform's mode. In QUATERNION mode the composed rotation is
     * stored as a quaternion straight from the delta — no euler decomposition,
     * so the drag never hits gimbal lock. In EULER mode it decomposes ZYX to the
     * representation nearest {@code referenceEuler}.
     *
     * @param deltaLocal     the delta rotation in the bone's parent frame (a pure
     *        rotation matrix); left untouched.
     * @param baseEuler      the grab euler stack the euler path composes onto.
     * @param referenceEuler the euler anchor of the readback — the GRAB euler for
     *        the free rotations (path-independent, self-recovering through the
     *        pole), the LIVE channels for the winding ring; the trade-off is on
     *        {@link #writeCompatibleEuler}.
     */
    public static void applyLocalDelta(DragContext ctx, Matrix3f deltaLocal, Vector3f baseEuler, Vector3f referenceEuler)
    {
        if (ctx.transform().rotationMode == Transform.RotationMode.QUATERNION)
        {
            Quaternionf delta = new Quaternionf().setFromNormalized(deltaLocal);

            ctx.writeRotationQuat(delta.mul(new Quaternionf(ctx.cache().quat)));
        }
        else
        {
            writeCompatibleEuler(ctx, new Matrix3f(deltaLocal).mul(eulerZYX(baseEuler)), referenceEuler);
        }
    }

    /**
     * World-direction &rarr; bone-parent-frame map from measured euler axes:
     * {@code parent^-1 = eulerAxes(source) * rotateAxes^-1}. {@code rotateAxes}
     * already folds in the parent and any model flips, so this recovers the
     * pure parent rotation; it is constant for the whole drag since the parent
     * doesn't move. Returns {@code null} when {@code rotateAxes} is degenerate.
     *
     * <p>Still used by the multi-bone pivot session, which captures per-bone
     * {@code rotateAxes} without a live {@link DragContext}; single-bone drags
     * use the euler-free {@link #parentInverse(DragContext, GizmoDrag)} instead.
     */
    public static Matrix3f computeParentInverse(Matrix3f rotateAxes, Vector3f sourceRadians)
    {
        Matrix3f rotateAxesInverse = new Matrix3f(rotateAxes);

        if (Math.abs(rotateAxesInverse.determinant()) < 1.0E-4F)
        {
            return null;
        }

        return eulerAxes(sourceRadians).mul(rotateAxesInverse.invert());
    }

    /**
     * Columns of the returned matrix are the (parent-frame) axes that
     * {@code rotate.x}, {@code rotate.y} and {@code rotate.z} rotate around for
     * the renderer's {@code Rz * Ry * Rx} order. They are orthonormal at rest
     * but skew as the bone turns, which is exactly why the decomposition has to
     * be re-evaluated against the live pose rather than a frozen snapshot.
     */
    public static Matrix3f eulerAxes(Vector3f rotateRadians)
    {
        Matrix3f axes = new Matrix3f();

        axes.setColumn(0, new Matrix3f().rotationZ(rotateRadians.z).rotateY(rotateRadians.y).transform(new Vector3f(1F, 0F, 0F)));
        axes.setColumn(1, new Matrix3f().rotationZ(rotateRadians.z).transform(new Vector3f(0F, 1F, 0F)));
        axes.setColumn(2, new Vector3f(0F, 0F, 1F));

        return axes;
    }
}
