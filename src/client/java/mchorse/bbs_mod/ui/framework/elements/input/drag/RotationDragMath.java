package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.ui.utils.GizmoDrag;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector2f;
import org.joml.Vector3f;

/**
 * The euler glue shared by every rotation drag: compose the turn as a
 * matrix, read ZYX euler angles back out, and write them unwrapped so a
 * value never jumps to a far 360°-equivalent mid-gesture. Used to be
 * copy-pasted across the view/trackball/arcball drags and the typed-angle
 * input.
 */
public final class RotationDragMath
{
    private RotationDragMath()
    {}

    /** The 360°-equivalent of {@code valueDeg} nearest to {@code referenceDeg}. */
    public static float unwrapDeg(float valueDeg, float referenceDeg)
    {
        return valueDeg + Math.round((referenceDeg - valueDeg) / 360F) * 360F;
    }

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
     * The cache's base euler for the parent-frame reconstruction
     * ({@link #computeParentInverse}). In quaternion mode the euler channels are
     * stale, so this returns the cache quaternion's ZYX equivalent — the exact
     * source {@link GizmoDrag#computeRotateAxes} perturbs, so the two stay
     * consistent. In euler mode it is the edited euler stack.
     */
    public static Vector3f cacheSourceEuler(DragContext ctx)
    {
        if (ctx.transform().rotationMode == Transform.RotationMode.QUATERNION)
        {
            return new Quaternionf(ctx.cache().quat).getEulerAnglesZYX(new Vector3f());
        }

        return ctx.cache().rotate;
    }

    /**
     * Read ZYX euler angles off {@code rotation} and write them into the
     * transform's rotate channel, each component unwrapped to the 360°-equivalent
     * nearest {@code referenceRadians}. The orientation stays continuous through
     * gimbal lock; only its euler representation jumps, which the unwrap hides.
     */
    public static void writeEulerUnwrapped(DragContext ctx, Matrix3f rotation, Vector3f referenceRadians)
    {
        Vector3f euler = rotation.getEulerAnglesZYX(new Vector3f());

        float rx = unwrapDeg(MathUtils.toDeg(euler.x), MathUtils.toDeg(referenceRadians.x));
        float ry = unwrapDeg(MathUtils.toDeg(euler.y), MathUtils.toDeg(referenceRadians.y));
        float rz = unwrapDeg(MathUtils.toDeg(euler.z), MathUtils.toDeg(referenceRadians.z));

        ctx.writeRotateDeg(rx, ry, rz);
    }

    /**
     * Compose a parent-frame delta rotation onto the grab base and write it per
     * the edited transform's mode. In QUATERNION mode the composed rotation is
     * stored as a quaternion straight from the delta — no euler decomposition,
     * so the drag never hits gimbal lock. In EULER mode it decomposes ZYX and
     * unwraps against the live value exactly as before.
     *
     * @param deltaLocal the delta rotation in the bone's parent frame (a pure
     *        rotation matrix); left untouched.
     * @param baseEuler  the grab euler stack the euler path composes onto.
     * @param liveEuler  the live euler stack the euler path unwraps against.
     */
    public static void applyLocalDelta(DragContext ctx, Matrix3f deltaLocal, Vector3f baseEuler, Vector3f liveEuler)
    {
        if (ctx.transform().rotationMode == Transform.RotationMode.QUATERNION)
        {
            Quaternionf delta = new Quaternionf().setFromNormalized(deltaLocal);

            ctx.writeRotationQuat(delta.mul(new Quaternionf(ctx.cache().quat)));
        }
        else
        {
            writeEulerUnwrapped(ctx, new Matrix3f(deltaLocal).mul(eulerZYX(baseEuler)), liveEuler);
        }
    }

    /**
     * World-direction &rarr; bone-parent-frame map captured at drag start:
     * {@code parent^-1 = eulerAxes(source) * rotateAxes^-1}. {@code rotateAxes}
     * already folds in the parent and any model flips, so this recovers the
     * pure parent rotation; it is constant for the whole drag since the parent
     * doesn't move. Returns {@code null} when {@code rotateAxes} is degenerate.
     */
    public static Matrix3f computeParentInverse(GizmoDrag drag, Vector3f sourceRadians)
    {
        return computeParentInverse(drag.rotateAxes, sourceRadians);
    }

    /** See {@link #computeParentInverse(GizmoDrag, Vector3f)}; takes the measured
     *  rotate axes directly so per-bone captures can use it without a drag. */
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
