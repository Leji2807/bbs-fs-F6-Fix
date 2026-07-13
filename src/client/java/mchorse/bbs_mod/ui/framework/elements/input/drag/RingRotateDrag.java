package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.ui.utils.GizmoDrag;
import mchorse.bbs_mod.utils.Axis;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Matrix3f;
import org.joml.Vector2f;
import org.joml.Vector3d;
import org.joml.Vector3f;

/**
 * Rotate around a single gizmo ring by the angle the cursor sweeps around
 * the projected gizmo center on screen. Unlike a ring-plane projection this
 * stays accurate when the ring faces the camera (where the plane hit
 * degenerates), and the per-frame deltas are unwrapped and accumulated
 * without limit so the user can wind several full turns in either direction.
 *
 * <p>The written angle is rebuilt every frame as the FIXED start value (the
 * cache) plus the total sweep — a pure function of the gesture. A single
 * euler component moves, so the angle canon does all the work here: winding
 * past 360° is just a growing number, no quaternion round-trip involved.
 */
public class RingRotateDrag extends DragStrategy
{
    /** World-space unit rotation axis of the grabbed ring, captured at drag start. */
    private final Vector3f axisDir = new Vector3f();

    /** Original unit ring direction (perpendicular to the rotation axis) captured at drag start. */
    private final Vector3f initialRingVec = new Vector3f();

    /** Projected gizmo origin in viewport pixels, captured at drag start. */
    private final Vector2f screenCenter = new Vector2f();

    /** Previous cursor angle (radians) around {@link #screenCenter}, unwrapped each frame. */
    private float lastScreenAngle;

    /** Maps screen-space angular motion to a rotation about the handle's axis (+1 or -1). */
    private float rotateSign = 1F;

    private float accumulatedDeg;

    /** Start rotation (degrees) from the cache; the fixed base the sweep adds onto. */
    private final Vector3f startRotateDeg = new Vector3f();

    /** Whether the edited bone stores its rotation as a quaternion. */
    private boolean quatMode;

    /** The space this gesture rotates in, captured at drag start. */
    private TransformSpace space = TransformSpace.LOCAL;

    /** For a non-local space: the ring's world axis expressed in the bone's parent
     *  frame (constant for the drag), so the world turn composes onto the pose the
     *  same gimbal-free way {@link ViewRotateDrag} does. Unused in LOCAL. */
    private final Vector3f axisLocalParent = new Vector3f();

    public RingRotateDrag(DragContext ctx, Axis axis)
    {
        super(ctx, TransformOp.ROTATE, axis, null);
    }

    @Override
    public Vector3f initialRingVec()
    {
        return this.initialRingVec;
    }

    @Override
    public float accumulatedRotateDeg()
    {
        return this.accumulatedDeg;
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
         * only moves the cursor reference; the axis, the base angles, the
         * pie's start edge and the swept angle survive so the gesture
         * continues instead of restarting. */
        if (this.hasStart)
        {
            this.lastScreenAngle = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);

            return;
        }

        this.space = this.ctx.space();

        /* The world axis this ring turns around, in the active space. LOCAL is
         * the renderer's actual rotation axis (GizmoDrag.computeRotateAxes) — not
         * the visible arrow, since cubic models post-multiply Ry(180°) and flip
         * bone-local X/Z; GLOBAL is the world axis; VIEW is the camera axis;
         * PARENT is the parent frame's axis (the frame the placement carries). */
        Vector3f axisDir = drag.rotationBasis(this.space).getColumn(this.axis.ordinal(), new Vector3f());

        if (axisDir.lengthSquared() < 1.0E-8F)
        {
            return;
        }

        axisDir.normalize();
        this.axisDir.set(axisDir);

        /* Screen-space ring rotation pivots around the gizmo origin projected to
         * the viewport. If the origin can't be projected (behind the camera)
         * there's nothing sensible to orbit around, so bail. */
        if (!drag.projectToScreen(drag.gizmoOrigin, this.screenCenter))
        {
            return;
        }

        this.lastScreenAngle = RotationDragMath.screenAngle(this.screenCenter, mouseX, mouseY);

        /* A positive rotation about +axis shows up as an increasing screen angle
         * (atan2 with Y pointing down) when the axis points away from the camera
         * (into the screen), and a decreasing one when it points back at it. */
        Vector3f intoScreen = new Vector3f(
            (float) (drag.gizmoOrigin.x - drag.cameraOrigin.x),
            (float) (drag.gizmoOrigin.y - drag.cameraOrigin.y),
            (float) (drag.gizmoOrigin.z - drag.cameraOrigin.z)
        );

        this.rotateSign = Math.signum(axisDir.dot(intoScreen));

        if (this.rotateSign == 0F)
        {
            this.rotateSign = 1F;
        }

        this.initialRingVec.set(this.computeStartRingVec(mouseX, mouseY, axisDir));
        this.accumulatedDeg = 0F;

        this.quatMode = this.ctx.transform().rotationMode == Transform.RotationMode.QUATERNION;

        /* The euler LOCAL ring adds its sweep onto the start angles; every other
         * path reads this as the pose the parent frame is recovered at (mode-aware,
         * so a quaternion bone uses its ZYX equivalent, not the stale channels). */
        Vector3f source = RotationDragMath.cacheSourceEuler(this.ctx);

        this.startRotateDeg.set(
            MathUtils.toDeg(source.x),
            MathUtils.toDeg(source.y),
            MathUtils.toDeg(source.z)
        );

        /* Every path except an euler LOCAL ring turns around a world axis, so map
         * it once into the bone's parent frame (constant for the drag) and apply
         * the turn as a parent-frame delta — ViewRotateDrag's gimbal-free path,
         * just axis-locked. Built from the measured rotateAxes (which fold in the
         * parent AND the cubic Ry(180) post-flip), NOT the raw bone orientation:
         * that flip is post-multiplied after the bone's own rotation, so the raw
         * orientation would leave X/Z turning backwards in GLOBAL/VIEW while the
         * euler LOCAL ring (also on rotateAxes) stays correct. */
        if (!this.channelPath())
        {
            Matrix3f parentInverse = RotationDragMath.parentInverse(this.ctx, drag);

            if (parentInverse == null)
            {
                return;
            }

            parentInverse.transform(this.axisDir, this.axisLocalParent);

            if (this.axisLocalParent.lengthSquared() < 1.0E-8F)
            {
                return;
            }

            this.axisLocalParent.normalize();
        }

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
        this.accumulatedDeg += MathUtils.toDeg(delta) * this.rotateSign;

        /* Common-pivot selection: hand the session the total world turn about
         * the ring's axis; it drives every selected bone, the primary included. */
        SelectionPivotSession session = this.ctx.pivotSession();

        if (session != null)
        {
            float sweepDeg = (float) this.snapValue(this.accumulatedDeg);

            session.applyRotation(new Matrix3f().rotation(MathUtils.toRad(sweepDeg), this.axisDir), true);

            return;
        }

        /* World-axis ring (GLOBAL/VIEW, or any quaternion ring): apply the swept
         * world turn about the ring's axis as a parent-frame delta onto the grab
         * pose (gimbal-free, quat- and euler-aware through applyLocalDelta). */
        if (!this.channelPath())
        {
            float sweepDeg = (float) this.snapValue(this.accumulatedDeg);
            Matrix3f deltaLocal = new Matrix3f().rotation(MathUtils.toRad(sweepDeg), this.axisLocalParent);

            RotationDragMath.applyLocalDelta(this.ctx, deltaLocal, this.ctx.cache().rotate, this.ctx.transform().rotate);

            return;
        }

        /* Euler LOCAL: bump the single driven channel. Snap only the driven axis,
         * and only in the written value — the raw accumulation stays smooth, and
         * the other two axes carry their start values and must not be rounded out
         * from under the user. */
        float rx = this.startRotateDeg.x;
        float ry = this.startRotateDeg.y;
        float rz = this.startRotateDeg.z;

        switch (this.axis)
        {
            case X: rx = (float) this.snapValue(rx + this.accumulatedDeg); break;
            case Y: ry = (float) this.snapValue(ry + this.accumulatedDeg); break;
            case Z: rz = (float) this.snapValue(rz + this.accumulatedDeg); break;
        }

        this.ctx.writeRotateDeg(rx, ry, rz);
    }

    /** The euler LOCAL ring is the only path that bumps a single euler channel
     *  (to keep &gt;360° winding); every other space and the quaternion mode turn
     *  about the ring's world axis through the gimbal-free parent-frame delta. */
    private boolean channelPath()
    {
        return this.space == TransformSpace.LOCAL && !this.quatMode;
    }

    private double snapValue(double value)
    {
        if (!this.ctx.shouldSnap(TransformOp.ROTATE))
        {
            return value;
        }

        return snap(value, BBSSettings.snapRotate.get());
    }

    /**
     * World-space ring direction (perpendicular to the rotation axis) at the
     * point where the drag started. Only feeds the rotation pie preview, so a
     * perpendicular fallback is acceptable when the cursor ray runs parallel to
     * the ring plane.
     */
    private Vector3f computeStartRingVec(int mouseX, int mouseY, Vector3f axisDir)
    {
        GizmoDrag drag = this.ctx.drag();
        Vector3f ring = new Vector3f();
        Vector3d hit = new Vector3d();

        if (drag.intersectPlane(mouseX, mouseY, axisDir, hit))
        {
            ring.set(
                (float) (hit.x - drag.gizmoOrigin.x),
                (float) (hit.y - drag.gizmoOrigin.y),
                (float) (hit.z - drag.gizmoOrigin.z)
            );

            float along = ring.dot(axisDir);

            ring.sub(new Vector3f(axisDir).mul(along));
        }

        if (ring.lengthSquared() < 1.0E-8F)
        {
            Vector3f fallback = Math.abs(axisDir.y) < 0.9F ? new Vector3f(0F, 1F, 0F) : new Vector3f(1F, 0F, 0F);

            axisDir.cross(fallback, ring);
        }

        return ring.normalize();
    }

    @Override
    public void applyNumeric(double value)
    {
        SelectionPivotSession session = this.ctx.pivotSession();

        if (session != null && this.hasStart)
        {
            session.applyRotation(new Matrix3f().rotation(MathUtils.toRad((float) value), this.axisDir), true);

            return;
        }

        /* A world-axis ring (GLOBAL/VIEW, or any quaternion ring) takes a numeric
         * turn as an axis rotation about the ring's world axis mapped to the
         * parent frame; only the euler LOCAL ring bumps a single channel. */
        if (!this.channelPath())
        {
            this.numericAxisRotation(value, this.axisLocalParent);

            return;
        }

        this.numericRotate(value);
    }

    @Override
    public String readout()
    {
        return String.format("%.1f°", this.accumulatedDeg);
    }
}
