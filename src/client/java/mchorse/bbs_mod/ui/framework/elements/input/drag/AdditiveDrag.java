package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.graphics.window.Window;
import mchorse.bbs_mod.utils.Axis;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Vector3f;

/**
 * The plain left/right additive drag: horizontal cursor travel nudges the
 * edited channel by a per-pixel step, with no 3D ray involved. Serves the
 * hotkey operations when ray dragging is disabled or no rendered gizmo is
 * available, and the uniform (three-axis) scale — whose centre grab reads
 * wildly through a single-axis ray lever but stays gentle here.
 */
public class AdditiveDrag extends DragStrategy
{
    /** Uniform (three-axis) scale: one lever drives every axis, like Ctrl. */
    private final boolean scaleAll;

    private int lastX;

    public AdditiveDrag(DragContext ctx, TransformOp op, Axis axis, Axis axis2, boolean scaleAll)
    {
        super(ctx, op, axis, axis2);

        this.scaleAll = scaleAll;
    }

    @Override
    public boolean isScaleAll()
    {
        return this.scaleAll;
    }

    /** Shift is damped through the step factor instead of the virtual cursor. */
    @Override
    public boolean usesFineCursor()
    {
        return false;
    }

    @Override
    public boolean acceptsNumeric()
    {
        return this.op != TransformOp.ROTATE || this.axis != null;
    }

    @Override
    public void begin(int mouseX, int mouseY)
    {
        this.lastX = mouseX;
        this.hasStart = true;
    }

    @Override
    public void update(int mouseX, int mouseY)
    {
        Transform transform = this.ctx.transform();

        if (transform == null)
        {
            return;
        }

        int dx = mouseX - this.lastX;

        this.lastX = mouseX;

        boolean all = this.op == TransformOp.SCALE && (this.scaleAll || Window.isCtrlPressed());
        float factor = this.ctx.additiveFactor(this.op) * (Window.isShiftPressed() ? FINE_DRAG_FACTOR : 1F);

        if (this.ctx.isLocal() && this.op == TransformOp.TRANSLATE)
        {
            Vector3f offset = this.ctx.localTranslateVector(factor * dx, this.axis);

            if (this.axis2 != null)
            {
                offset.add(this.ctx.localTranslateVector(factor * dx, this.axis2));
            }

            Vector3f live = transform.translate;

            this.ctx.writeTranslate(live.x + offset.x, live.y + offset.y, live.z + offset.z);

            return;
        }

        Vector3f live = this.liveValue(transform);
        Vector3f value = new Vector3f(live);

        if (this.op == TransformOp.ROTATE)
        {
            value.mul(180F / MathUtils.PI);
        }

        if (this.axis == Axis.X || all) value.x += factor * dx;
        if (this.axis == Axis.Y || all) value.y += factor * dx;
        if (this.axis == Axis.Z || all) value.z += factor * dx;
        if (!all && this.axis2 == Axis.X) value.x += factor * dx;
        if (!all && this.axis2 == Axis.Y) value.y += factor * dx;
        if (!all && this.axis2 == Axis.Z) value.z += factor * dx;

        switch (this.op)
        {
            case TRANSLATE:
                this.ctx.writeTranslate(value.x, value.y, value.z);
                break;
            case SCALE:
                this.ctx.writeScale(value.x, value.y, value.z);
                break;
            default:
                this.ctx.writeRotateDeg(value.x, value.y, value.z);
                break;
        }
    }

    private Vector3f liveValue(Transform transform)
    {
        if (this.op == TransformOp.SCALE)
        {
            return transform.scale;
        }

        if (this.op == TransformOp.ROTATE)
        {
            return transform.rotate;
        }

        return transform.translate;
    }

    @Override
    public void applyNumeric(double value)
    {
        switch (this.op)
        {
            case TRANSLATE:
                this.numericTranslate(value);
                break;
            case SCALE:
                this.numericScale(value, this.scaleAll || Window.isCtrlPressed());
                break;
            default:
                this.numericRotate(value);
                break;
        }
    }

    @Override
    public String readout()
    {
        Transform transform = this.ctx.transform();
        Transform cache = this.ctx.cache();

        if (this.op == TransformOp.TRANSLATE)
        {
            return this.axisDeltaReadout(new Vector3f(transform.translate).sub(cache.translate), false);
        }

        if (this.op == TransformOp.SCALE)
        {
            return this.axisDeltaReadout(new Vector3f(transform.scale).sub(cache.scale), this.scaleAll);
        }

        if (this.axis == null)
        {
            return null;
        }

        Vector3f now = transform.rotate;
        Vector3f start = cache.rotate;
        float delta = this.axis == Axis.X ? now.x - start.x : (this.axis == Axis.Y ? now.y - start.y : now.z - start.z);

        return String.format("%.1f°", MathUtils.toDeg(delta));
    }
}
