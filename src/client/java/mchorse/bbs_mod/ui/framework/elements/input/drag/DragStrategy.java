package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.graphics.window.Window;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.utils.Axis;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.joml.Matrices;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * One live transform gesture: a single operation, in a single control style
 * (axis ray, screen plane, ring sweep, trackball, ...), owning all of its
 * per-gesture state. The host editor creates a strategy when an edit starts
 * ({@link DragStrategyFactory}), feeds it the cursor each frame, and drops it
 * when the edit ends — so no state leaks between modes or between gestures.
 *
 * <p>{@link #begin} anchors the gesture at the cursor and is re-invoked on
 * every cursor wrap, so implementations must be idempotent with respect to
 * their accumulated amounts (fold or keep them; never reset the user's
 * progress).
 */
public abstract class DragStrategy
{
    /** Cursor-speed multiplier for a drag gesture while Shift is held (precision drag). */
    public static final float FINE_DRAG_FACTOR = 0.1F;

    /** Factor the modifier keys apply to a gizmo step: Ctrl coarsens, Alt refines. */
    protected static final float STEP_MODIFIER = 5F;

    /** Base degrees of view-axis roll per mouse-wheel notch while sphere-dragging. */
    protected static final float TRACKBALL_WHEEL_DEG = 5F;

    protected final DragContext ctx;
    protected final TransformOp op;
    protected final Axis axis;
    protected final Axis axis2;

    /** Whether {@link #begin} anchored successfully and {@link #update} may run. */
    protected boolean hasStart;

    protected DragStrategy(DragContext ctx, TransformOp op, Axis axis, Axis axis2)
    {
        this.ctx = ctx;
        this.op = op;
        this.axis = axis;
        this.axis2 = axis2;
    }

    public final TransformOp op()
    {
        return this.op;
    }

    /** Anchor the gesture at the cursor. Re-invoked on cursor wraps. */
    public abstract void begin(int mouseX, int mouseY);

    /** Advance the gesture from the cursor position. */
    public abstract void update(int mouseX, int mouseY);

    /** Apply a typed numeric amount on top of the cached start transform. */
    public abstract void applyNumeric(double value);

    /**
     * Whether the host should feed the Shift-slowed virtual cursor. The
     * additive fallback damps Shift through its own factor instead, so it
     * reads the raw cursor.
     */
    public boolean usesFineCursor()
    {
        return true;
    }

    /** Consume a mouse-wheel event (depth move, sphere roll). */
    public boolean scroll(UIContext context)
    {
        return false;
    }

    /** On-screen summary of what the gesture changed so far, or {@code null}. */
    public String readout()
    {
        return null;
    }

    public boolean isSphere()
    {
        return false;
    }

    public boolean isView()
    {
        return false;
    }

    public boolean isScreenTranslate()
    {
        return false;
    }

    public boolean isScaleAll()
    {
        return false;
    }

    /** Special label for the editing chip row, or {@code null} for axis letters. */
    public String editingTargetLabel()
    {
        return null;
    }

    /** Whether typed numeric input may drive this gesture. */
    public boolean acceptsNumeric()
    {
        return true;
    }

    /** Aim the typed angle while on the sphere (X/Y keys); {@code false} elsewhere. */
    public boolean handleNumericAxisKey(int key)
    {
        return false;
    }

    /** Prefix of the numeric card while typing on the sphere. */
    public String numericPrefix()
    {
        return "";
    }

    /* Pie-preview data the gizmo renders during a rotation gesture. Default
     * to "nothing swept" so only the gestures that own a pie override. */

    /** Swept angle (degrees) of the rotation pie; {@code 0} when the gesture has none. */
    public float accumulatedRotateDeg()
    {
        return 0F;
    }

    /** Ring direction the pie starts from, or {@code null} when the gesture has none. */
    public Vector3f initialRingVec()
    {
        return null;
    }

    /** Screen-space start edge of the view sweep pie (radians, Y-down convention). */
    public float viewGrabScreenAngle()
    {
        return 0F;
    }

    /** Signed screen-space span of the view sweep, in radians. */
    public float viewScreenSweepRad()
    {
        return 0F;
    }

    /**
     * Apply the shared modifier keys to a gizmo step amount: Alt makes it fine
     * (÷{@value #STEP_MODIFIER}), Ctrl makes it coarse (×{@value #STEP_MODIFIER}),
     * both/neither leave it as-is.
     */
    protected static float applyStepModifiers(float step)
    {
        if (Window.isAltPressed()) step /= STEP_MODIFIER;
        if (Window.isCtrlPressed()) step *= STEP_MODIFIER;

        return step;
    }

    protected static double snap(double value, float step)
    {
        return step <= 0F ? value : Math.round(value / step) * (double) step;
    }

    protected static void appendAxis(StringBuilder builder, String label, float value)
    {
        if (builder.length() > 0)
        {
            builder.append("  ");
        }

        builder.append(label).append(' ').append(String.format("%+.3f", value));
    }

    /** Per-axis delta readout filtered to this gesture's axes (or all of them). */
    protected String axisDeltaReadout(Vector3f delta, boolean allAxes)
    {
        StringBuilder builder = new StringBuilder();

        if (allAxes || this.axis == Axis.X || this.axis2 == Axis.X) appendAxis(builder, "X", delta.x);
        if (allAxes || this.axis == Axis.Y || this.axis2 == Axis.Y) appendAxis(builder, "Y", delta.y);
        if (allAxes || this.axis == Axis.Z || this.axis2 == Axis.Z) appendAxis(builder, "Z", delta.z);

        return builder.length() == 0 ? null : builder.toString();
    }

    /** Per-axis rotation deltas (degrees) of a free rotation, measured from the cache. */
    protected String freeRotateReadout()
    {
        Vector3f start = this.ctx.cache().rotate;
        Vector3f now = this.ctx.transform().rotate;

        return String.format("X %+.1f°  Y %+.1f°  Z %+.1f°",
            MathUtils.toDeg(now.x - start.x),
            MathUtils.toDeg(now.y - start.y),
            MathUtils.toDeg(now.z - start.z));
    }

    /* Typed numeric amounts, applied on top of the cached start transform.
     * Shared here because the ray strategies and the additive fallback use
     * the exact same semantics: an offset for translate (units) and rotate
     * (degrees), a factor for scale. */

    /** The typed offset in translate-channel units, honoring the local toggle and the gesture's axes. */
    protected Vector3f numericTranslateOffset(double value)
    {
        if (this.ctx.isLocal())
        {
            Vector3f offset = this.ctx.localTranslateVector(value, this.axis);

            if (this.axis2 != null)
            {
                offset.add(this.ctx.localTranslateVector(value, this.axis2));
            }

            return offset;
        }

        Vector3f offset = new Vector3f();

        if (this.axis == Axis.X || this.axis2 == Axis.X) offset.x = (float) value;
        if (this.axis == Axis.Y || this.axis2 == Axis.Y) offset.y = (float) value;
        if (this.axis == Axis.Z || this.axis2 == Axis.Z) offset.z = (float) value;

        return offset;
    }

    protected void numericTranslate(double value)
    {
        Transform cache = this.ctx.cache();
        Vector3f offset = this.numericTranslateOffset(value);

        this.ctx.writeTranslate(
            cache.translate.x + offset.x,
            cache.translate.y + offset.y,
            cache.translate.z + offset.z
        );
    }

    protected void numericScale(double value, boolean all)
    {
        Transform cache = this.ctx.cache();
        Vector3f s = new Vector3f(cache.scale);

        if (all || this.axis == Axis.X || this.axis2 == Axis.X) s.x = (float) (cache.scale.x * value);
        if (all || this.axis == Axis.Y || this.axis2 == Axis.Y) s.y = (float) (cache.scale.y * value);
        if (all || this.axis == Axis.Z || this.axis2 == Axis.Z) s.z = (float) (cache.scale.z * value);

        this.ctx.writeScale(s.x, s.y, s.z);
    }

    protected void numericRotate(double value)
    {
        boolean quatMode = this.ctx.transform().rotationMode == Transform.RotationMode.QUATERNION;

        /* In quaternion mode the euler channels are stale, so read the base off
         * the cache quaternion (its ZYX equivalent), bump the axis, and store
         * the result back as a quaternion — the single-axis add is exact, so
         * this stays gimbal-safe. */
        Vector3f source = quatMode
            ? new Quaternionf(this.ctx.cache().quat).getEulerAnglesZYX(new Vector3f())
            : this.ctx.cache().rotate;

        float rx = MathUtils.toDeg(source.x);
        float ry = MathUtils.toDeg(source.y);
        float rz = MathUtils.toDeg(source.z);

        if (this.axis == Axis.X || this.axis2 == Axis.X) rx += value;
        if (this.axis == Axis.Y || this.axis2 == Axis.Y) ry += value;
        if (this.axis == Axis.Z || this.axis2 == Axis.Z) rz += value;

        if (quatMode) this.ctx.writeRotationQuat(Matrices.toQuaternionZYXDegrees(rx, ry, rz));
        else this.ctx.writeRotateDeg(rx, ry, rz);
    }

    /**
     * Premultiply the start orientation (the cache) by a turn of the typed
     * degrees about a fixed parent-frame axis, then read the euler angles
     * back — the same composition the cursor-driven view/trackball drags
     * use, but from a single exact angle.
     */
    protected void numericAxisRotation(double degrees, Vector3f localAxis)
    {
        if (localAxis.lengthSquared() < 1.0E-8F)
        {
            return;
        }

        Vector3f source = this.ctx.cache().rotate;

        RotationDragMath.applyLocalDelta(
            this.ctx,
            new Matrix3f().rotation(MathUtils.toRad((float) degrees), localAxis),
            source, source
        );
    }
}
