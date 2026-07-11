package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.ui.UIKeys;
import mchorse.bbs_mod.ui.utils.GizmoDrag;
import mchorse.bbs_mod.utils.MathUtils;
import org.joml.Matrix3f;
import org.joml.Vector2f;
import org.joml.Vector3f;

/**
 * View-plane rotation: the axis is fixed to the gizmo-to-camera direction,
 * and the angle comes from sweeping the cursor around the projected gizmo
 * center, exactly like a per-axis ring. Unlike a single ring, the resulting
 * world-space turn is spread across all three rotate components, so it stays
 * "common" to the three axes.
 *
 * <p>Rather than solving per-frame euler deltas (which the matrix inverse
 * makes unstable near gimbal lock — a full 360 sweep passes through it and
 * twitches), each step premultiplies the live rotation matrix by the spin
 * and reads the euler angles back out. The orientation stays continuous
 * through gimbal; only its euler representation jumps, which is invisible.
 */
public class ViewRotateDrag extends DragStrategy
{
    /** The axis points at the camera (out of the screen), so an increasing
     *  screen angle (clockwise, Y down) is a negative turn about it. */
    private static final float ROTATE_SIGN = -1F;

    /** View axis expressed in the bone's parent frame, captured at drag start. */
    private final Vector3f viewLocalAxis = new Vector3f();

    /** Projected gizmo origin in viewport pixels, captured at drag start. */
    private final Vector2f screenCenter = new Vector2f();

    private float lastScreenAngle;

    /** Cursor angle (radians, screen convention) at the moment the drag began —
     *  the fixed start edge of the view sweep pie. */
    private float grabScreenAngle;

    private float accumulatedDeg;

    private boolean gizmoSpace;

    public ViewRotateDrag(DragContext ctx)
    {
        super(ctx, TransformOp.ROTATE, null, null);
    }

    @Override
    public boolean isView()
    {
        return true;
    }

    @Override
    public String editingTargetLabel()
    {
        return UIKeys.TRANSFORMS_TARGET_VIEW.get();
    }

    @Override
    public float viewGrabScreenAngle()
    {
        return this.grabScreenAngle;
    }

    @Override
    public float accumulatedRotateDeg()
    {
        return this.accumulatedDeg;
    }

    /** The screen angle winds opposite to the applied turn, hence the {@link #ROTATE_SIGN} fold. */
    @Override
    public float viewScreenSweepRad()
    {
        return MathUtils.toRad(this.accumulatedDeg) * ROTATE_SIGN;
    }

    @Override
    public void begin(int mouseX, int mouseY)
    {
        GizmoDrag drag = this.ctx.drag();

        if (drag == null || this.ctx.transform() == null)
        {
            this.hasStart = false;

            return;
        }

        Vector3f viewAxis = new Vector3f(
            (float) (drag.cameraOrigin.x - drag.gizmoOrigin.x),
            (float) (drag.cameraOrigin.y - drag.gizmoOrigin.y),
            (float) (drag.cameraOrigin.z - drag.gizmoOrigin.z)
        );

        if (viewAxis.lengthSquared() < 1.0E-8F || !drag.projectToScreen(drag.gizmoOrigin, this.screenCenter))
        {
            this.hasStart = false;

            return;
        }

        viewAxis.normalize();
        this.lastScreenAngle = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);
        this.grabScreenAngle = this.lastScreenAngle;
        this.accumulatedDeg = 0;

        this.gizmoSpace = this.ctx.isGizmoSpace();

        /* Express the view axis once in the bone's parent frame; it stays
         * constant for the whole drag, while each step premultiplies the live
         * rotation by a turn about it. */
        Vector3f source = this.gizmoSpace ? this.ctx.transform().rotate2 : this.ctx.transform().rotate;
        Matrix3f parentInverse = RotationDragMath.computeParentInverse(drag, source);

        if (parentInverse == null)
        {
            this.hasStart = false;

            return;
        }

        parentInverse.transform(viewAxis, this.viewLocalAxis);

        if (this.viewLocalAxis.lengthSquared() < 1.0E-8F)
        {
            this.hasStart = false;

            return;
        }

        this.viewLocalAxis.normalize();
        this.hasStart = true;
    }

    @Override
    public void update(int mouseX, int mouseY)
    {
        if (!this.hasStart || this.ctx.transform() == null)
        {
            return;
        }

        float current = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);
        float delta = RotationDragMath.wrapSeamRad(current - this.lastScreenAngle);

        this.lastScreenAngle = current;

        float angle = delta * ROTATE_SIGN;

        if (angle == 0F)
        {
            return;
        }

        this.accumulatedDeg += MathUtils.toDeg(angle);

        Vector3f source = this.gizmoSpace ? this.ctx.transform().rotate2 : this.ctx.transform().rotate;

        Matrix3f composed = new Matrix3f()
            .rotation(angle, this.viewLocalAxis)
            .mul(RotationDragMath.eulerZYX(source));

        RotationDragMath.writeEulerUnwrapped(this.ctx, this.gizmoSpace, composed, source);
    }

    @Override
    public void applyNumeric(double value)
    {
        this.numericAxisRotation(value, this.viewLocalAxis, this.gizmoSpace);
    }

    @Override
    public String readout()
    {
        return this.freeRotateReadout(this.gizmoSpace);
    }
}
