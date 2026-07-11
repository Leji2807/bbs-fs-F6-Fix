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
 * <p>The rotation is rebuilt every frame from the FIXED start orientation
 * (the cache) plus the total swept angle about the view axis anchored at
 * grab time — a pure function of the gesture, never of the previous frame's
 * euler readback. The euler angles are read out once per frame in the one
 * shared place ({@link RotationDragMath#writeEulerUnwrapped}); the
 * orientation stays continuous through gimbal lock, only its euler
 * representation jumps, which the unwrap hides.
 */
public class ViewRotateDrag extends DragStrategy
{
    /** The axis points at the camera (out of the screen), so an increasing
     *  screen angle (clockwise, Y down) is a negative turn about it. */
    private static final float ROTATE_SIGN = -1F;

    /** View axis expressed in the bone's parent frame, captured at drag start. */
    private final Vector3f viewLocalAxis = new Vector3f();

    /** The same axis in world space, for the common-pivot selection session. */
    private final Vector3f viewWorldAxis = new Vector3f();

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

        /* A re-anchor (cursor wrap, cursor control resumed after typed input)
         * only moves the cursor reference; the anchored axis, the pie's start
         * edge and the swept angle survive so the gesture continues instead
         * of restarting. */
        if (this.hasStart)
        {
            this.lastScreenAngle = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);

            return;
        }

        Vector3f viewAxis = new Vector3f(
            (float) (drag.cameraOrigin.x - drag.gizmoOrigin.x),
            (float) (drag.cameraOrigin.y - drag.gizmoOrigin.y),
            (float) (drag.cameraOrigin.z - drag.gizmoOrigin.z)
        );

        if (viewAxis.lengthSquared() < 1.0E-8F || !drag.projectToScreen(drag.gizmoOrigin, this.screenCenter))
        {
            return;
        }

        viewAxis.normalize();
        this.viewWorldAxis.set(viewAxis);

        this.gizmoSpace = this.ctx.isGizmoSpace();

        /* Express the view axis once in the bone's parent frame, mapped at the
         * start orientation (the cache) it will be composed against; it stays
         * constant for the whole drag. */
        Vector3f source = this.gizmoSpace ? this.ctx.cache().rotate2 : this.ctx.cache().rotate;
        Matrix3f parentInverse = RotationDragMath.computeParentInverse(drag, source);

        if (parentInverse == null)
        {
            return;
        }

        parentInverse.transform(viewAxis, this.viewLocalAxis);

        if (this.viewLocalAxis.lengthSquared() < 1.0E-8F)
        {
            return;
        }

        this.viewLocalAxis.normalize();
        this.lastScreenAngle = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);
        this.grabScreenAngle = this.lastScreenAngle;
        this.accumulatedDeg = 0F;
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

        if (delta == 0F)
        {
            return;
        }

        this.accumulatedDeg += MathUtils.toDeg(delta * ROTATE_SIGN);

        /* Common-pivot selection: the total world sweep about the view axis
         * drives every selected bone through the session. */
        SelectionPivotSession session = this.ctx.pivotSession();

        if (session != null)
        {
            session.applyRotation(new Matrix3f().rotation(MathUtils.toRad(this.accumulatedDeg), this.viewWorldAxis));

            return;
        }

        Vector3f base = this.gizmoSpace ? this.ctx.cache().rotate2 : this.ctx.cache().rotate;
        Vector3f live = this.gizmoSpace ? this.ctx.transform().rotate2 : this.ctx.transform().rotate;

        Matrix3f deltaLocal = new Matrix3f().rotation(MathUtils.toRad(this.accumulatedDeg), this.viewLocalAxis);

        RotationDragMath.applyLocalDelta(this.ctx, this.gizmoSpace, deltaLocal, base, live);
    }

    @Override
    public void applyNumeric(double value)
    {
        SelectionPivotSession session = this.ctx.pivotSession();

        if (session != null && this.hasStart)
        {
            session.applyRotation(new Matrix3f().rotation(MathUtils.toRad((float) value), this.viewWorldAxis));

            return;
        }

        this.numericAxisRotation(value, this.viewLocalAxis, this.gizmoSpace);
    }

    @Override
    public String readout()
    {
        return this.freeRotateReadout(this.gizmoSpace);
    }
}
