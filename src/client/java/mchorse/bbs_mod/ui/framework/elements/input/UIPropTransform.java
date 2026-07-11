package mchorse.bbs_mod.ui.framework.elements.input;

import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.graphics.window.Window;
import mchorse.bbs_mod.l10n.keys.IKey;
import mchorse.bbs_mod.settings.values.IValueListener;
import mchorse.bbs_mod.settings.values.IValueNotifier;
import mchorse.bbs_mod.settings.values.ui.ValueOrder;
import mchorse.bbs_mod.ui.Keys;
import mchorse.bbs_mod.ui.UIKeys;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.ui.framework.elements.UIElement;
import mchorse.bbs_mod.ui.framework.elements.events.UITrackpadDragEndEvent;
import mchorse.bbs_mod.ui.framework.elements.utils.FontRenderer;
import mchorse.bbs_mod.ui.framework.elements.input.drag.DragContext;
import mchorse.bbs_mod.ui.framework.elements.input.drag.DragStrategy;
import mchorse.bbs_mod.ui.framework.elements.input.drag.DragStrategyFactory;
import mchorse.bbs_mod.ui.framework.elements.input.drag.PivotMode;
import mchorse.bbs_mod.ui.framework.elements.input.drag.SelectionPivotSession;
import mchorse.bbs_mod.ui.framework.elements.input.drag.TransformNumericInput;
import mchorse.bbs_mod.ui.framework.elements.input.drag.TransformOp;
import mchorse.bbs_mod.ui.utils.Gizmo;
import mchorse.bbs_mod.ui.utils.GizmoDrag;
import mchorse.bbs_mod.ui.utils.UIUtils;
import mchorse.bbs_mod.ui.utils.keys.KeyAction;
import mchorse.bbs_mod.ui.utils.icons.Icons;
import mchorse.bbs_mod.utils.Axis;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.Timer;
import mchorse.bbs_mod.utils.colors.Colors;
import mchorse.bbs_mod.utils.pose.Transform;
import net.minecraft.client.MinecraftClient;
import org.joml.Matrix3f;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFW;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Transform editor that drives the gizmo and hotkey (G/S/R) edits. The
 * editor itself is a thin coordinator: it owns the edit session (what is
 * being edited, the start snapshot, accept/reject) while all per-gesture
 * state and math live in the active {@link DragStrategy}, created through
 * {@link DragStrategyFactory} when an edit starts and dropped when it ends.
 */
public class UIPropTransform extends UITransform
{
    private static final double[] CURSOR_X = new double[1];
    private static final double[] CURSOR_Y = new double[1];

    private static final Vector3f ZERO_RING_VEC = new Vector3f();

    private Transform transform;
    private Runnable preCallback;
    private Runnable postCallback;
    private Runnable endCallback;

    private boolean editing;
    private Axis axis = Axis.X;
    private Axis axis2;
    private Transform cache = new Transform();
    private Timer checker = new Timer(30);

    private boolean model;
    private boolean local;

    /** Drag snapshot the active gesture works against (kept for the gizmo's pie preview). */
    private GizmoDrag drag;
    private boolean hotkeyMode;
    private Supplier<GizmoDrag> hotkeyDragSupplier;

    /** The live gesture; non-null exactly while {@link #editing}. */
    private DragStrategy strategy;
    private final DragContext bridge = new Bridge();

    /** Common-pivot session driving a multi-bone edit, or {@code null} when the
     *  edit is single-bone / per-channel (see {@link #createPivotSession()}). */
    private SelectionPivotSession pivotSession;

    private final TransformNumericInput numeric = new TransformNumericInput();

    /* Fine-drag (Shift) precision: a virtual cursor that lags the real one,
     * advancing at {@link DragStrategy#FINE_DRAG_FACTOR} speed while Shift is
     * held, so every ray gesture slows uniformly without per-mode code. The
     * lag is the accumulated offset between the two. */
    private float fineOffsetX;
    private float fineOffsetY;
    private int fineLastX;
    private int fineLastY;
    private boolean fineHasLast;

    private UITransformHandler handler;

    public UIPropTransform()
    {
        this.handler = new UITransformHandler(this);
        this.local = BBSSettings.defaultLocalTransform.get();

        this.context((menu) ->
        {
            menu.action(
                this.local ? Icons.FULLSCREEN : Icons.MINIMIZE,
                this.local ? UIKeys.TRANSFORMS_CONTEXT_SWITCH_GLOBAL : UIKeys.TRANSFORMS_CONTEXT_SWITCH_LOCAL,
                this::toggleLocal
            );

            menu.actions.add(0, menu.actions.remove(menu.actions.size() - 1));
        });

        this.iconT.callback = (b) -> this.toggleLocal();
        this.iconT.hoverColor = Colors.LIGHTEST_GRAY;
        this.iconT.setEnabled(true);
        this.updateLocalUI();

        /* Bone-selection editors get the pivot mode switch on the rotation
         * row's icon (otherwise a decorative placeholder). */
        if (this.supportsPivotModes())
        {
            this.iconR.callback = (b) -> this.cyclePivotMode();
            this.iconR.hoverColor = Colors.LIGHTEST_GRAY;
            this.iconR.setEnabled(true);
            this.updatePivotUI();
        }

        /* Each finished value-field drag closes the current undo block, so dragging a
         * field several times in a row undoes one drag at a time (see endGesture). */
        for (UITrackpad field : new UITrackpad[]{this.tx, this.ty, this.tz, this.sx, this.sy, this.sz, this.rx, this.ry, this.rz, this.r2x, this.r2y, this.r2z})
        {
            field.getEvents().register(UITrackpadDragEndEvent.class, (e) -> this.endGesture());
        }

        this.noCulling();
    }

    public UIPropTransform callbacks(Supplier<IValueNotifier> notifier)
    {
        return this.callbacks(
            () -> notifier.get().preNotify(),
            () -> notifier.get().postNotify(),
            () -> notifier.get().preNotify(IValueListener.FLAG_UNMERGEABLE)
        );
    }

    public UIPropTransform callbacks(Runnable pre, Runnable post)
    {
        return this.callbacks(pre, post, null);
    }

    public UIPropTransform callbacks(Runnable pre, Runnable post, Runnable end)
    {
        this.preCallback = pre;
        this.postCallback = post;
        this.endCallback = end;

        return this;
    }

    public void preCallback()
    {
        if (this.preCallback != null) this.preCallback.run();
    }

    public void postCallback()
    {
        if (this.postCallback != null) this.postCallback.run();
    }

    /**
     * Close the current undo block so the next transform gesture starts a fresh,
     * separately-undoable entry. Fired at each gesture boundary — a value-field drag
     * end and the gizmo commit — rather than per value change, so one continuous drag
     * still merges into a single undo while consecutive drags stay distinct.
     */
    public void endGesture()
    {
        if (this.endCallback != null) this.endCallback.run();
    }

    public void setModel()
    {
        this.model = true;
    }

    public UIPropTransform hotkeyDrag(Supplier<GizmoDrag> supplier)
    {
        this.hotkeyDragSupplier = supplier;

        return this;
    }

    public boolean isLocal()
    {
        return this.local;
    }

    @Override
    protected Transform getEditedTransform()
    {
        return this.transform;
    }

    public Axis getAxis2()
    {
        return this.axis2;
    }

    public boolean isScreenTranslate()
    {
        return this.strategy != null && this.strategy.isScreenTranslate();
    }

    /** Old-logic no-op: kept so hosts that gave the spaces bar a backdrop still compile. */
    public UIPropTransform barBackground()
    {
        return this;
    }

    protected boolean supportsMirror()
    {
        return false;
    }

    /** Whether this editor drives a bone selection that supports the common-pivot
     *  modes (the pivot switch icon and hotkey only show up there). */
    protected boolean supportsPivotModes()
    {
        return false;
    }

    private void cyclePivotMode()
    {
        BBSSettings.pivotMode.set(PivotMode.current().next().ordinal());
        this.updatePivotUI();
        UIUtils.playClick();
    }

    private void updatePivotUI()
    {
        PivotMode mode = PivotMode.current();
        IKey label = mode == PivotMode.MEDIAN ? UIKeys.TRANSFORMS_PIVOT_MEDIAN
            : (mode == PivotMode.ACTIVE ? UIKeys.TRANSFORMS_PIVOT_ACTIVE : UIKeys.TRANSFORMS_PIVOT_INDIVIDUAL);

        this.iconR.tooltip(label);
    }

    public boolean isMirrorEdit()
    {
        return BBSSettings.poseMirrorEdit.get();
    }

    public boolean isAlternateInvert()
    {
        return BBSSettings.poseAlternateInvert.get();
    }

    private void toggleLocal()
    {
        this.local = !this.local;

        if (!this.local && this.transform != null)
        {
            this.fillT(this.transform.translate.x, this.transform.translate.y, this.transform.translate.z);
        }

        this.updateLocalUI();
    }

    private void updateLocalUI()
    {
        this.tx.forcedLabel(this.local ? UIKeys.GENERAL_X : null);
        this.ty.forcedLabel(this.local ? UIKeys.GENERAL_Y : null);
        this.tz.forcedLabel(this.local ? UIKeys.GENERAL_Z : null);
        this.tx.relative(this.local);
        this.ty.relative(this.local);
        this.tz.relative(this.local);
        this.iconT.tooltip(this.local ? UIKeys.TRANSFORMS_CONTEXT_SWITCH_GLOBAL : UIKeys.TRANSFORMS_CONTEXT_SWITCH_LOCAL);
    }

    private Vector3f calculateLocalVector(double factor, Axis axis)
    {
        if (this.transform == null)
        {
            return new Vector3f();
        }

        Vector3f vector3f = new Vector3f(
            (float) (axis == Axis.X ? factor : 0D),
            (float) (axis == Axis.Y ? factor : 0D),
            (float) (axis == Axis.Z ? factor : 0D)
        );
        /* I have no fucking idea why I have to rotate it 180 degrees by X axis... but it works! */
        Matrix3f matrix = new Matrix3f()
            .rotateX(this.model ? MathUtils.PI : 0F)
            .mul(this.transform.createRotationMatrix());

        matrix.transform(vector3f);

        return vector3f;
    }

    public UIPropTransform enableHotkeys()
    {
        return this.enableHotkeys(() -> true);
    }

    public UIPropTransform enableHotkeys(Supplier<Boolean> enabled)
    {
        IKey category = UIKeys.TRANSFORMS_KEYS_CATEGORY;
        Supplier<Boolean> active = () -> enabled.get() && this.editing;

        this.keys().register(Keys.TRANSFORMATIONS_TRANSLATE, () -> this.enableMode(TransformOp.TRANSLATE)).active(enabled).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_SCALE, () -> this.enableMode(TransformOp.SCALE)).active(enabled).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_ROTATE, () -> this.enableMode(TransformOp.ROTATE)).active(enabled).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_COMBINED, () -> Gizmo.INSTANCE.toggleCombined()).strict().active(enabled).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_X, () -> this.setEditingAxis(Axis.X)).active(active).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_Y, () -> this.setEditingAxis(Axis.Y)).active(active).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_Z, () -> this.setEditingAxis(Axis.Z)).active(active).category(category);
        this.keys().register(Keys.TRANSFORMATIONS_TOGGLE_LOCAL, () ->
        {
            this.toggleLocal();
            UIUtils.playClick();
        }).active(enabled).category(category);

        if (this.supportsPivotModes())
        {
            this.keys().register(Keys.TRANSFORMATIONS_PIVOT_MODE, this::cyclePivotMode).active(enabled).category(category);
        }

        return this;
    }

    public Transform getTransform()
    {
        return this.transform;
    }

    public boolean isEditing()
    {
        return this.editing;
    }

    public Axis getAxis()
    {
        return this.axis;
    }

    /** The active edit's operation, or {@code null} when nothing is being edited. */
    public TransformOp getOp()
    {
        return this.strategy == null ? null : this.strategy.op();
    }

    /**
     * The live gesture driving the edit, or {@code null}. Every (re)start —
     * including an axis switch mid-edit — builds a fresh instance, so the
     * gizmo uses its identity to scope per-gesture state (the ring freeze).
     */
    public DragStrategy getStrategy()
    {
        return this.strategy;
    }

    /** Whether the active rotation is one of the sphere's kinds (trackball or arcball). */
    public boolean isSphereRotate()
    {
        return this.strategy != null && this.strategy.isSphere();
    }

    public boolean isViewRotate()
    {
        return this.strategy != null && this.strategy.isView();
    }

    /** Whether the active scale drives all three axes off one lever (centre scale
     *  handle or an unconstrained S). Distinct from {@link #isUniformScale()}, which
     *  is the trackpad's scale-field linking. */
    public boolean isScaleAll()
    {
        return this.strategy != null && this.strategy.isScaleAll();
    }

    public Vector3f getInitialDragRingVec()
    {
        Vector3f vec = this.strategy == null ? null : this.strategy.initialRingVec();

        return vec == null ? ZERO_RING_VEC : vec;
    }

    public float getAccumulatedRotateDeg()
    {
        return this.strategy == null ? 0F : this.strategy.accumulatedRotateDeg();
    }

    /** Screen-space start edge of the view sweep pie (radians, Y-down convention). */
    public float getViewGrabScreenAngle()
    {
        return this.strategy == null ? 0F : this.strategy.viewGrabScreenAngle();
    }

    /** Signed screen-space span of the view sweep, in radians. */
    public float getViewScreenSweepRad()
    {
        return this.strategy == null ? 0F : this.strategy.viewScreenSweepRad();
    }

    /**
     * A short summary of what the active drag has changed so far, for the gizmo's
     * on-screen readout: degrees for a rotation (axis or view ring by swept angle,
     * the 3D sphere by net turn), the per-axis offset for a move, the per-axis
     * factor delta for a scale. Returns {@code null} when there is nothing to show.
     */
    public String getDragReadout()
    {
        if (!this.editing || this.transform == null || this.strategy == null)
        {
            return null;
        }

        return this.strategy.readout();
    }

    public GizmoDrag getDrag()
    {
        return this.drag;
    }

    public int getDebugLineStencilIndex()
    {
        if (!this.editing || this.isScreenTranslate())
        {
            return -1;
        }

        if (this.axis2 != null)
        {
            if ((this.axis == Axis.X && this.axis2 == Axis.Z) || (this.axis == Axis.Z && this.axis2 == Axis.X))
            {
                return Gizmo.STENCIL_XZ;
            }

            if ((this.axis == Axis.X && this.axis2 == Axis.Y) || (this.axis == Axis.Y && this.axis2 == Axis.X))
            {
                return Gizmo.STENCIL_XY;
            }

            if ((this.axis == Axis.Z && this.axis2 == Axis.Y) || (this.axis == Axis.Y && this.axis2 == Axis.Z))
            {
                return Gizmo.STENCIL_ZY;
            }
        }

        if (this.axis == Axis.X) return Gizmo.STENCIL_X;
        if (this.axis == Axis.Y) return Gizmo.STENCIL_Y;
        if (this.axis == Axis.Z) return Gizmo.STENCIL_Z;

        return -1;
    }

    public void refillTransform()
    {
        this.setTransform(this.getTransform());
    }

    public void setTransform(Transform transform)
    {
        this.transform = transform;

        if (transform == null)
        {
            this.disable();
            this.fillT(0, 0, 0);
            this.fillS(1, 1, 1);
            this.fillR(0, 0, 0);
            this.fillR2(0, 0, 0);

            return;
        }

        float minScale = Math.min(transform.scale.x, Math.min(transform.scale.y, transform.scale.z));
        float maxScale = Math.max(transform.scale.x, Math.max(transform.scale.y, transform.scale.z));

        if (BBSSettings.uniformScale.get())
        {
            if (
                (minScale == maxScale && !this.isUniformScale()) ||
                (minScale != maxScale && this.isUniformScale())
            ) {
                this.toggleUniformScale();
            }
        }

        this.fillT(transform.translate.x, transform.translate.y, transform.translate.z);
        this.fillS(transform.scale.x, transform.scale.y, transform.scale.z);
        this.fillR(MathUtils.toDeg(transform.rotate.x), MathUtils.toDeg(transform.rotate.y), MathUtils.toDeg(transform.rotate.z));
        this.fillR2(MathUtils.toDeg(transform.rotate2.x), MathUtils.toDeg(transform.rotate2.y), MathUtils.toDeg(transform.rotate2.z));
    }

    /* Edit entry points. The mouse path (a gizmo handle pick) supplies the
     * axes directly and never switches the gizmo's display mode; the keyboard
     * path walks the user-configured hotkey orders and switches the displayed
     * handles on the first press. Both funnel into startEdit. */

    public void enableMode(TransformOp op)
    {
        GizmoDrag drag = this.getHotkeyDrag();
        boolean ray = BBSSettings.transformHotkeys3dRay.get() && drag != null;

        /* The scale key defaults to a uniform three-axis scale (Blender-style);
         * X/Y/Z then constrain it to one axis via setEditingAxis. */
        if (op == TransformOp.SCALE)
        {
            this.enableUniformScale(drag, true);

            return;
        }

        /* G/S/R walk their handles in the user-configured order (the
         * *_hotkey_order settings), wrapping past the end back to the first
         * step. Steps whose handle is unavailable drop out: the ray-driven
         * ones without a rendered gizmo, the sphere when it's turned off. */
        HotkeyTarget target = this.nextHotkeyTarget(op, ray);

        if (target == HotkeyTarget.VIEW)
        {
            this.enableViewRotate(drag, true);
        }
        else if (target == HotkeyTarget.SPHERE)
        {
            this.enableSphereRotate(drag, true);
        }
        else if (target == HotkeyTarget.SCREEN)
        {
            this.enableScreenTranslate(drag, true);
        }
        else
        {
            this.enableHotkeyAxis(op, target.axis, drag);
        }
    }

    /** The walk step the active edit corresponds to ({@code null} when not editing this op). */
    private HotkeyTarget currentHotkeyTarget(TransformOp op)
    {
        if (!this.editing || this.getOp() != op)
        {
            return null;
        }

        if (this.isViewRotate()) return HotkeyTarget.VIEW;
        if (this.isSphereRotate()) return HotkeyTarget.SPHERE;
        if (this.isScreenTranslate()) return HotkeyTarget.SCREEN;
        if (this.axis == Axis.Y) return HotkeyTarget.Y;
        if (this.axis == Axis.Z) return HotkeyTarget.Z;

        return HotkeyTarget.X;
    }

    private HotkeyTarget nextHotkeyTarget(TransformOp op, boolean ray)
    {
        ValueOrder order = op == TransformOp.TRANSLATE ? BBSSettings.translateHotkeyOrder : (op == TransformOp.SCALE ? BBSSettings.scaleHotkeyOrder : BBSSettings.rotateHotkeyOrder);
        List<HotkeyTarget> steps = new ArrayList<>();

        for (String token : order.get())
        {
            HotkeyTarget target = HotkeyTarget.byToken(token);

            if (target == null || (target.needsRay && !ray))
            {
                continue;
            }

            if (target == HotkeyTarget.SPHERE && !BBSSettings.rotate3dSphere.get())
            {
                continue;
            }

            steps.add(target);
        }

        if (steps.isEmpty())
        {
            return HotkeyTarget.X;
        }

        int index = steps.indexOf(this.currentHotkeyTarget(op));

        return steps.get((index + 1) % steps.size());
    }

    /**
     * Start (or switch to) a hotkey-driven operation along a specific axis.
     * Unlike the mouse path this keeps the hotkey semantics (numeric input,
     * accept/reject overlay, the display-mode switch on the first press);
     * the axis comes from the configured hotkey order rather than a fixed
     * cycle.
     */
    private void enableHotkeyAxis(TransformOp op, Axis axis, GizmoDrag drag)
    {
        if (this.switchGizmoDisplayMode(op))
        {
            return;
        }

        this.startEdit(op, axis, null, DragStrategyFactory.Variant.AXIS, drag, true);
    }

    public void enableMode(TransformOp op, Axis axis)
    {
        this.enableMode(op, axis, null, null);
    }

    public void enableMode(TransformOp op, Axis axis, Axis axis2)
    {
        this.enableMode(op, axis, axis2, null);
    }

    /**
     * Start an operation from a mouse handle pick: the axes come straight
     * from the picked handle, so this never cycles and never switches the
     * gizmo's display mode. The keyboard path goes through
     * {@link #enableMode(TransformOp)} and the configured hotkey orders instead.
     */
    public void enableMode(TransformOp op, Axis axis, Axis axis2, GizmoDrag drag)
    {
        this.startEdit(op, axis == null ? Axis.X : axis, axis2, DragStrategyFactory.Variant.AXIS, drag, axis == null);
    }

    public void enableSphereRotate(GizmoDrag drag)
    {
        this.enableSphereRotate(drag, false);
    }

    /** Start whichever free rotation the sphere is configured to drive. */
    public void enableSphereRotate(GizmoDrag drag, boolean hotkeyMode)
    {
        if (BBSSettings.rotate3dSphereMode.get() == 1) this.enableArcball(drag, hotkeyMode);
        else this.enableTrackball(drag, hotkeyMode);
    }

    public void enableTrackball(GizmoDrag drag)
    {
        this.enableTrackball(drag, false);
    }

    public void enableTrackball(GizmoDrag drag, boolean hotkeyMode)
    {
        if (hotkeyMode && this.switchGizmoDisplayMode(TransformOp.ROTATE))
        {
            return;
        }

        this.startEdit(TransformOp.ROTATE, null, null, DragStrategyFactory.Variant.TRACKBALL, drag, hotkeyMode);
    }

    public void enableArcball(GizmoDrag drag)
    {
        this.enableArcball(drag, false);
    }

    public void enableArcball(GizmoDrag drag, boolean hotkeyMode)
    {
        if (hotkeyMode && this.switchGizmoDisplayMode(TransformOp.ROTATE))
        {
            return;
        }

        this.startEdit(TransformOp.ROTATE, null, null, DragStrategyFactory.Variant.ARCBALL, drag, hotkeyMode);
    }

    public void enableViewRotate(GizmoDrag drag)
    {
        this.enableViewRotate(drag, false);
    }

    public void enableViewRotate(GizmoDrag drag, boolean hotkeyMode)
    {
        if (hotkeyMode && this.switchGizmoDisplayMode(TransformOp.ROTATE))
        {
            return;
        }

        this.startEdit(TransformOp.ROTATE, null, null, DragStrategyFactory.Variant.VIEW, drag, hotkeyMode);
    }

    /**
     * Start a uniform (three-axis) scale: one lever axis drives all three, the
     * same math Ctrl+axis-scale uses. A mouse pick ({@code hotkeyMode == false})
     * never switches the gizmo's display mode; as the S-key walk step it switches
     * to scale mode on the first press like the other hotkey starters.
     */
    public void enableUniformScale(GizmoDrag drag)
    {
        this.enableUniformScale(drag, false);
    }

    public void enableUniformScale(GizmoDrag drag, boolean hotkeyMode)
    {
        if (hotkeyMode && this.switchGizmoDisplayMode(TransformOp.SCALE))
        {
            return;
        }

        this.startEdit(TransformOp.SCALE, Axis.X, null, DragStrategyFactory.Variant.UNIFORM_SCALE, drag, hotkeyMode);
    }

    /**
     * Start a screen-space (view-plane) translate: the object moves along the
     * camera's right/up axes in the plane facing the camera. Grabbing the
     * centre cube with the mouse never switches the gizmo's display mode
     * (like the other handle picks); as a hotkey walk step the first press
     * switches it like the rest of the hotkey starters.
     */
    public void enableScreenTranslate(GizmoDrag drag)
    {
        this.enableScreenTranslate(drag, false);
    }

    public void enableScreenTranslate(GizmoDrag drag, boolean hotkeyMode)
    {
        if (hotkeyMode && this.switchGizmoDisplayMode(TransformOp.TRANSLATE))
        {
            return;
        }

        this.startEdit(TransformOp.TRANSLATE, Axis.X, Axis.Y, DragStrategyFactory.Variant.SCREEN, drag, hotkeyMode);
    }

    /**
     * The hotkey starters switch the gizmo's displayed handles to their
     * operation on the first press; when that happens the press is consumed
     * by the switch and no edit starts. In combined mode there is nothing to
     * switch, so the edit always starts.
     */
    private boolean switchGizmoDisplayMode(TransformOp op)
    {
        Gizmo.Mode target = op == TransformOp.TRANSLATE ? Gizmo.Mode.TRANSLATE : (op == TransformOp.SCALE ? Gizmo.Mode.SCALE : Gizmo.Mode.ROTATE);

        return Gizmo.INSTANCE.getMode() != Gizmo.Mode.COMBINED && Gizmo.INSTANCE.setMode(target);
    }

    /**
     * The one edit-start ritual every entry point funnels into: close any
     * previous edit, snapshot the transform, build the strategy for the
     * request and anchor it at the cursor, then raise the accept/reject
     * overlay.
     */
    private void startEdit(TransformOp op, Axis axis, Axis axis2, DragStrategyFactory.Variant variant, GizmoDrag drag, boolean hotkeyMode)
    {
        UIContext context = this.getContext();

        if (context == null || this.transform == null)
        {
            return;
        }

        this.numeric.clear();

        if (this.editing)
        {
            this.restore();
        }

        this.editing = true;
        this.axis = axis;
        this.axis2 = axis2;
        this.hotkeyMode = hotkeyMode;
        this.drag = drag;

        this.cache.copy(this.transform);
        Gizmo.INSTANCE.trackTransform(this);

        /* A common-pivot multi-bone session only makes sense with a world-space
         * drag context; the additive fallback has no world delta to feed it. */
        this.pivotSession = drag == null ? null : this.createPivotSession();

        this.strategy = DragStrategyFactory.create(this.bridge, op, axis, axis2, variant, hotkeyMode);
        this.strategy.begin(context.mouseX, context.mouseY);

        if (!this.handler.hasParent())
        {
            context.menu.overlay.add(this.handler);
        }
    }

    private GizmoDrag getHotkeyDrag()
    {
        return this.hotkeyDragSupplier == null ? null : this.hotkeyDragSupplier.get();
    }

    /**
     * Build the common-pivot session for the edit that is starting, or
     * {@code null} to keep the per-channel path. The base editor edits a
     * single transform, so there is never a selection to pivot; the delta
     * (multi-bone) editors override this with their selection capture.
     */
    protected SelectionPivotSession createPivotSession()
    {
        return null;
    }

    /**
     * Constrain the live edit to an axis (or, with Shift, to the plane
     * perpendicular to it): rewind to the start values and rebuild the
     * gesture as a plain axis drag of the same operation.
     */
    private void setEditingAxis(Axis axis)
    {
        if (Window.isShiftPressed())
        {
            switch (axis)
            {
                case X:
                    this.axis = Axis.Y;
                    this.axis2 = Axis.Z;
                    break;
                case Y:
                    this.axis = Axis.Z;
                    this.axis2 = Axis.X;
                    break;
                case Z:
                    this.axis = Axis.X;
                    this.axis2 = Axis.Y;
                    break;
            }
        }
        else
        {
            this.axis = axis;
            this.axis2 = null;
        }

        if (!this.editing)
        {
            return;
        }

        TransformOp op = this.getOp();

        this.restore();

        UIContext context = this.getContext();

        if (context != null && op != null)
        {
            this.strategy = DragStrategyFactory.create(this.bridge, op, this.axis, this.axis2, DragStrategyFactory.Variant.AXIS, this.hotkeyMode);
            this.strategy.begin(context.mouseX, context.mouseY);
        }

        /* Re-route an in-progress typed amount onto the freshly picked axis. */
        if (this.numeric.isActive())
        {
            this.applyNumericInput();
        }
    }

    /** Rewind every channel to the values captured when the edit began. */
    private void restore()
    {
        /* A pivot session moved every bone by its own amount, so the rewind
         * must go through its per-bone snapshots — the per-channel setters
         * below would fan the PRIMARY's delta onto the whole selection. */
        if (this.pivotSession != null)
        {
            this.pivotSession.restore();

            return;
        }

        this.setT(null, this.cache.translate.x, this.cache.translate.y, this.cache.translate.z);
        this.setS(null, this.cache.scale.x, this.cache.scale.y, this.cache.scale.z);
        this.setR(null, MathUtils.toDeg(this.cache.rotate.x), MathUtils.toDeg(this.cache.rotate.y), MathUtils.toDeg(this.cache.rotate.z));
        this.setR2(null, MathUtils.toDeg(this.cache.rotate2.x), MathUtils.toDeg(this.cache.rotate2.y), MathUtils.toDeg(this.cache.rotate2.z));
    }

    private void disable()
    {
        this.editing = false;
        this.axis2 = null;
        this.hotkeyMode = false;
        this.strategy = null;
        this.drag = null;
        this.pivotSession = null;
        this.fineHasLast = false;
        this.numeric.clear();
        Gizmo.INSTANCE.clearTrackedTransform(this);

        if (this.handler.hasParent())
        {
            this.handler.removeFromParent();
        }
    }

    public void acceptChanges()
    {
        this.disable();
        this.setTransform(this.transform);
        this.endGesture();
    }

    public void rejectChanges()
    {
        if (this.transform == null)
        {
            this.disable();

            return;
        }

        /* Rewind BEFORE tearing down: restore() routes a pivot-session revert
         * through the session's per-bone snapshots, and disable() nulls that
         * session. Do it the other way round and the rewind falls back to the
         * per-channel path, which fans the primary's values onto the whole
         * selection — the bones come back crooked instead of where they were. */
        this.restore();
        this.disable();
        this.setTransform(this.transform);
    }

    /** Route a wheel event into the live gesture (depth move, sphere roll). */
    public boolean scrollDrag(UIContext context)
    {
        return this.editing && this.transform != null && this.strategy != null && this.strategy.scroll(context);
    }

    /* Numeric (keyboard) input for hotkey-driven transforms */

    /**
     * Numeric input only rides on the GSR keyboard operations ({@link #hotkeyMode}),
     * never on a mouse handle drag; the active gesture additionally has a say
     * (the screen-space grab spreads one drag across two camera axes, so a
     * single typed scalar is ambiguous there).
     */
    private boolean acceptsNumericInput()
    {
        return this.editing && this.hotkeyMode && this.transform != null
            && this.strategy != null && this.strategy.acceptsNumeric();
    }

    /**
     * Feed one key into the live numeric buffer: digits and the decimal point
     * extend it, {@code -} flips the sign, backspace trims it (and hands control
     * back to the cursor once everything is erased). Returns whether the key was
     * consumed as numeric input.
     */
    private boolean handleNumericInputKey(UIContext context)
    {
        if (!this.acceptsNumericInput())
        {
            return false;
        }

        KeyAction action = context.getKeyAction();

        if (action != KeyAction.PRESSED && action != KeyAction.REPEAT)
        {
            return false;
        }

        int key = context.getKeyCode();

        /* While typing on the sphere, X/Y aim the typed angle at the
         * horizontal (screen-up axis) or vertical (screen-right axis) turn.
         * Without typed digits they must fall through to the axis keybinds
         * and constrain to a ring — otherwise they read as dead keys. */
        if (this.numeric.isActive() && this.strategy.handleNumericAxisKey(key))
        {
            this.applyNumericInput();

            return true;
        }

        switch (this.numeric.feedKey(key))
        {
            case EMPTIED:
                this.stopNumericInput(context);

                return true;

            case CHANGED:
                this.applyNumericInput();

                return true;

            case CONSUMED:
                return true;

            default:
                return false;
        }
    }

    /**
     * Erasing the whole buffer cancels numeric mode: rewind to the operation's
     * start and re-anchor the cursor drag at the current pointer so mouse
     * control resumes without a jump.
     */
    private void stopNumericInput(UIContext context)
    {
        this.numeric.clear();
        this.restore();

        /* The cursor was free to roam while typing; re-anchor the precision
         * tracking here so the resumed drag doesn't inherit a stale lag. */
        this.resetFineCursor(context.mouseX, context.mouseY);

        if (this.strategy != null)
        {
            this.strategy.begin(context.mouseX, context.mouseY);
        }

        this.setTransform(this.transform);
    }

    /** Recompute the transform from the start snapshot plus the typed amount. */
    private void applyNumericInput()
    {
        if (this.transform == null || this.strategy == null)
        {
            return;
        }

        this.strategy.applyNumeric(this.numeric.value());
        this.setTransform(this.transform);
    }

    @Override
    protected void internalSetT(double x, Axis axis)
    {
        if (this.transform == null)
        {
            return;
        }

        if (this.local)
        {
            try
            {
                Vector3f vector3f = this.calculateLocalVector(x, axis);

                this.setT(null,
                    this.transform.translate.x + vector3f.x,
                    this.transform.translate.y + vector3f.y,
                    this.transform.translate.z + vector3f.z
                );
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }
        else
        {
            super.internalSetT(x, axis);
        }
    }

    @Override
    public void setT(Axis axis, double x, double y, double z)
    {
        if (this.transform == null)
        {
            return;
        }

        this.preCallback();
        this.transform.translate.set((float) x, (float) y, (float) z);
        this.postCallback();
    }

    @Override
    public void setS(Axis axis, double x, double y, double z)
    {
        if (this.transform == null)
        {
            return;
        }

        this.preCallback();
        this.transform.scale.set((float) x, (float) y, (float) z);
        this.postCallback();
    }

    @Override
    public void setR(Axis axis, double x, double y, double z)
    {
        if (this.transform == null)
        {
            return;
        }

        this.preCallback();
        this.transform.rotate.set(MathUtils.toRad((float) x), MathUtils.toRad((float) y), MathUtils.toRad((float) z));
        this.postCallback();
    }

    @Override
    public void setR2(Axis axis, double x, double y, double z)
    {
        if (this.transform == null)
        {
            return;
        }

        this.preCallback();
        this.transform.rotate2.set(MathUtils.toRad((float) x), MathUtils.toRad((float) y), MathUtils.toRad((float) z));
        this.postCallback();
    }

    @Override
    protected boolean subKeyPressed(UIContext context)
    {
        if (this.editing)
        {
            if (context.isPressed(GLFW.GLFW_KEY_ENTER))
            {
                this.acceptChanges();

                return true;
            }
            else if (context.isPressed(GLFW.GLFW_KEY_ESCAPE))
            {
                this.rejectChanges();

                return true;
            }
            else if (this.handleNumericInputKey(context))
            {
                return true;
            }
        }

        return super.subKeyPressed(context);
    }

    /** Short label of what the active drag grabs: axis letters, the screen
     *  plane, the view ring, or one of the sphere's rotations. */
    private String editingTargetLabel()
    {
        String special = this.strategy == null ? null : this.strategy.editingTargetLabel();

        if (special != null)
        {
            return special;
        }

        if (this.getOp() == TransformOp.SCALE && (this.isScaleAll() || Window.isCtrlPressed()))
        {
            return "XYZ";
        }

        String label = this.axis == null ? "" : this.axis.name();

        if (this.axis2 != null)
        {
            label += this.axis2.name();
        }

        return label;
    }

    /** Axis letters tint to their gizmo colors; everything else stays white. */
    private int editingTargetColor()
    {
        boolean singleAxis = this.axis != null && this.axis2 == null
            && !this.isScreenTranslate()
            && !(this.getOp() == TransformOp.SCALE && (this.isScaleAll() || Window.isCtrlPressed()));

        if (!singleAxis)
        {
            return Colors.WHITE;
        }

        if (this.axis == Axis.X) return Colors.A100 | Colors.RED;
        if (this.axis == Axis.Y) return Colors.A100 | Colors.GREEN;

        return Colors.A100 | Colors.BLUE;
    }

    /** Local/global chip; scale ignores the space toggle, so it gets none. */
    private String editingSpaceLabel()
    {
        if (this.getOp() == TransformOp.SCALE)
        {
            return null;
        }

        return (this.local ? UIKeys.TRANSFORMS_SPACE_LOCAL : UIKeys.TRANSFORMS_SPACE_GLOBAL).get();
    }

    /** The live vector of the edited channel, for the cursor's value card. */
    private Vector3f getValue()
    {
        if (this.transform == null)
        {
            return new Vector3f();
        }

        TransformOp op = this.getOp();

        if (op == TransformOp.SCALE)
        {
            return this.transform.scale;
        }
        else if (op == TransformOp.ROTATE)
        {
            return this.local && BBSSettings.gizmos.get() ? this.transform.rotate2 : this.transform.rotate;
        }

        return this.transform.translate;
    }

    /**
     * Maintain the virtual cursor for the current frame. While Shift is held it
     * advances at {@link DragStrategy#FINE_DRAG_FACTOR} of the real cursor — the
     * rest of the motion piles into the lag offset; released, it tracks the
     * cursor 1:1 again with no jump. Ray gestures read {@link #fineX}/{@link #fineY}
     * so they all slow uniformly without any per-mode code.
     */
    private void updateFineCursor(int mouseX, int mouseY)
    {
        if (!this.fineHasLast)
        {
            this.resetFineCursor(mouseX, mouseY);

            return;
        }

        if (Window.isShiftPressed())
        {
            float keep = 1F - DragStrategy.FINE_DRAG_FACTOR;

            this.fineOffsetX += (mouseX - this.fineLastX) * keep;
            this.fineOffsetY += (mouseY - this.fineLastY) * keep;
        }

        this.fineLastX = mouseX;
        this.fineLastY = mouseY;
    }

    private void resetFineCursor(int mouseX, int mouseY)
    {
        this.fineOffsetX = 0F;
        this.fineOffsetY = 0F;
        this.fineLastX = mouseX;
        this.fineLastY = mouseY;
        this.fineHasLast = true;
    }

    private int fineX(int mouseX)
    {
        return Math.round(mouseX - this.fineOffsetX);
    }

    private int fineY(int mouseY)
    {
        return Math.round(mouseY - this.fineOffsetY);
    }

    /**
     * Advance the live gesture: wrap the cursor at the window edges (re-anchoring
     * the strategy at the teleported position) and feed the strategy the cursor —
     * virtual (Shift-slowed) for ray gestures, raw for the additive fallback,
     * which damps Shift through its step factor instead.
     */
    private void updateDrag(UIContext context)
    {
        /* UIContext.mouseX can't be used because when cursor is outside of window
         * its position stops being updated. That's why it has to be queried manually
         * through GLFW...
         *
         * It gets updated outside the window only when one of mouse buttons is
         * being held! */
        GLFW.glfwGetCursorPos(Window.getWindow(), CURSOR_X, CURSOR_Y);

        MinecraftClient mc = MinecraftClient.getInstance();
        int w = mc.getWindow().getWidth();

        double rawX = CURSOR_X[0];
        double fx = Math.ceil(w / (double) context.menu.width);
        int border = 5;
        int borderPadding = border + 1;

        this.updateFineCursor(context.mouseX, context.mouseY);

        if (rawX <= border || rawX >= w - border)
        {
            int wrapX;

            if (rawX <= border)
            {
                Window.moveCursor(w - borderPadding, (int) mc.mouse.getY());
                wrapX = context.menu.width - (int) (borderPadding / fx);
            }
            else
            {
                Window.moveCursor(borderPadding, (int) mc.mouse.getY());
                wrapX = (int) (borderPadding / fx);
            }

            this.checker.mark();

            /* The wrap re-anchors the drag at the teleported position, so the
             * virtual cursor resets there too — no lag carries across the seam. */
            this.resetFineCursor(wrapX, context.mouseY);

            if (this.strategy != null)
            {
                this.strategy.begin(wrapX, context.mouseY);
            }

            return;
        }

        if (this.strategy != null)
        {
            if (this.strategy.usesFineCursor())
            {
                this.strategy.update(this.fineX(context.mouseX), this.fineY(context.mouseY));
            }
            else
            {
                this.strategy.update(context.mouseX, context.mouseY);
            }
        }

        this.setTransform(this.transform);
    }

    @Override
    public void render(UIContext context)
    {
        if (this.editing && !this.numeric.isActive() && this.checker.isTime())
        {
            this.updateDrag(context);
        }

        super.render(context);

        if (this.editing)
        {
            FontRenderer font = context.batcher.getFont();
            TransformOp editOp = this.getOp();
            String op = (editOp == TransformOp.TRANSLATE ? UIKeys.TRANSFORMS_TRANSLATE : editOp == TransformOp.SCALE ? UIKeys.TRANSFORMS_SCALE : UIKeys.TRANSFORMS_ROTATE).get();
            String target = this.editingTargetLabel();
            String space = this.editingSpaceLabel();

            /* Chip row: the operation on the primary color, then what is
             * grabbed (axis letters in their gizmo colors), then the editing
             * space. The 5s account for textCard's box overhang at the
             * default card offset. */
            int gap = 2;
            int rowWidth = font.getWidth(op) + 5 + gap + font.getWidth(target) + 5;

            if (space != null)
            {
                rowWidth += gap + font.getWidth(space) + 5;
            }

            int x = this.area.mx(rowWidth) + 3;
            int y = this.area.my(font.getHeight());

            context.batcher.textCard(op, x, y, Colors.WHITE, BBSSettings.primaryColor(Colors.A50));
            x += font.getWidth(op) + 5 + gap;
            context.batcher.textCard(target, x, y, this.editingTargetColor(), Colors.A50);

            if (space != null)
            {
                x += font.getWidth(target) + 5 + gap;
                context.batcher.textCard(space, x, y, Colors.LIGHTEST_GRAY, Colors.A50);
            }

            /* Label echoed both at the cursor and (when typing) under the info row. */
            String numericLabel = null;

            if (this.axis != null)
            {
                Vector3f v = this.getValue();
                float val = this.axis == Axis.X ? v.x : (this.axis == Axis.Y ? v.y : v.z);

                if (editOp == TransformOp.ROTATE)
                {
                    val = MathUtils.toDeg(val);
                }

                String valueLabel = String.format(java.util.Locale.US, "%.2f", val);

                if (this.axis2 != null)
                {
                    float val2 = this.axis2 == Axis.X ? v.x : (this.axis2 == Axis.Y ? v.y : v.z);

                    if (editOp == TransformOp.ROTATE)
                    {
                        val2 = MathUtils.toDeg(val2);
                    }

                    valueLabel += ", " + String.format(java.util.Locale.US, "%.2f", val2);
                }

                /* While typing, lead with the raw input so the user sees exactly
                 * what they've entered, with the resulting value in parentheses. */
                String cursorLabel = this.numeric.isActive()
                    ? this.numeric.display() + " (" + valueLabel + ")"
                    : valueLabel;

                if (this.numeric.isActive())
                {
                    numericLabel = cursorLabel;
                }

                context.batcher.textCard(cursorLabel, context.mouseX + 12, context.mouseY + 12, Colors.WHITE, Colors.A50);
            }
            else if (this.numeric.isActive())
            {
                /* The view ring and the sphere have no single axis component to
                 * echo, so show the typed angle, plus the aimed direction. */
                String prefix = this.strategy == null ? "" : this.strategy.numericPrefix();

                numericLabel = prefix + this.numeric.display() + "°";

                context.batcher.textCard(numericLabel, context.mouseX + 12, context.mouseY + 12, Colors.WHITE, Colors.A50);
            }

            /* Mirror the live numeric input on its own card right under the info row. */
            if (numericLabel != null)
            {
                int nx = this.area.mx(font.getWidth(numericLabel));
                int ny = y + font.getHeight() + 8;

                context.batcher.textCard(numericLabel, nx, ny, Colors.WHITE, BBSSettings.primaryColor(Colors.A50));
            }
        }
    }

    /**
     * A step of a transform hotkey's walk. Tokens match the entries of the
     * translate/scale/rotate hotkey order settings.
     */
    public enum HotkeyTarget
    {
        VIEW("view", null, true),
        SPHERE("sphere", null, true),
        SCREEN("screen", null, true),
        X("x", Axis.X, false),
        Y("y", Axis.Y, false),
        Z("z", Axis.Z, false);

        public final String token;
        public final Axis axis;
        /** Whether the step is driven by the 3D ray and so needs a rendered gizmo. */
        public final boolean needsRay;

        HotkeyTarget(String token, Axis axis, boolean needsRay)
        {
            this.token = token;
            this.axis = axis;
            this.needsRay = needsRay;
        }

        public static HotkeyTarget byToken(String token)
        {
            for (HotkeyTarget target : values())
            {
                if (target.token.equals(token))
                {
                    return target;
                }
            }

            return null;
        }
    }

    /**
     * Bridge the active {@link DragStrategy} works through: it exposes the
     * edit session's state and funnels every write back through the editor's
     * virtual {@code setT/setS/setR/setR2}, so the delta editors keep fanning
     * edits onto their selections.
     */
    private class Bridge implements DragContext
    {
        @Override
        public Transform transform()
        {
            return UIPropTransform.this.transform;
        }

        @Override
        public Transform cache()
        {
            return UIPropTransform.this.cache;
        }

        @Override
        public GizmoDrag drag()
        {
            return UIPropTransform.this.drag;
        }

        @Override
        public void setDrag(GizmoDrag drag)
        {
            UIPropTransform.this.drag = drag;
        }

        @Override
        public GizmoDrag freshHotkeyDrag()
        {
            return UIPropTransform.this.getHotkeyDrag();
        }

        @Override
        public SelectionPivotSession pivotSession()
        {
            return UIPropTransform.this.pivotSession;
        }

        @Override
        public boolean isLocal()
        {
            return UIPropTransform.this.local;
        }

        @Override
        public boolean isModel()
        {
            return UIPropTransform.this.model;
        }

        @Override
        public boolean isGizmoSpace()
        {
            return UIPropTransform.this.local && BBSSettings.gizmos.get();
        }

        /* Blender-style snapping: every gesture is free by default and snaps to
         * the configured step only while Ctrl is held. Typed numeric input is
         * exact already, so it never snaps. */
        @Override
        public boolean shouldSnap(TransformOp op)
        {
            return UIPropTransform.this.editing && UIPropTransform.this.getOp() == op
                && Window.isCtrlPressed() && !UIPropTransform.this.numeric.isActive();
        }

        @Override
        public float additiveFactor(TransformOp op)
        {
            UITrackpad reference = op == TransformOp.TRANSLATE ? UIPropTransform.this.tx : (op == TransformOp.SCALE ? UIPropTransform.this.sx : UIPropTransform.this.rx);

            return (float) reference.getValueModifier();
        }

        @Override
        public Vector3f localTranslateVector(double factor, Axis axis)
        {
            return UIPropTransform.this.calculateLocalVector(factor, axis);
        }

        @Override
        public float sphereWorldRadius()
        {
            return Gizmo.INSTANCE.getSphereWorldRadius();
        }

        @Override
        public void refreshFields()
        {
            UIPropTransform.this.setTransform(UIPropTransform.this.transform);
        }

        @Override
        public void writeTranslate(float x, float y, float z)
        {
            UIPropTransform.this.setT(null, x, y, z);
        }

        @Override
        public void writeScale(float x, float y, float z)
        {
            UIPropTransform.this.setS(null, x, y, z);
        }

        @Override
        public void writeRotateDeg(float xDeg, float yDeg, float zDeg)
        {
            UIPropTransform.this.setR(null, xDeg, yDeg, zDeg);
        }

        @Override
        public void writeRotate2Deg(float xDeg, float yDeg, float zDeg)
        {
            UIPropTransform.this.setR2(null, xDeg, yDeg, zDeg);
        }
    }

    public static class UITransformHandler extends UIElement
    {
        private UIPropTransform transform;

        public UITransformHandler(UIPropTransform transform)
        {
            this.transform = transform;
        }

        @Override
        protected boolean subMouseClicked(UIContext context)
        {
            if (this.transform.editing)
            {
                if (context.mouseButton == 0)
                {
                    this.transform.acceptChanges();

                    return true;
                }
                else if (context.mouseButton == 1)
                {
                    this.transform.rejectChanges();

                    return true;
                }
            }

            return super.subMouseClicked(context);
        }

        @Override
        protected boolean subMouseScrolled(UIContext context)
        {
            /* While sphere-dragging the wheel rolls about the view axis; during a
             * screen-space grab it drives depth; otherwise it keeps adjusting
             * the drag sensitivity amplifier as before. */
            if (this.transform.scrollDrag(context))
            {
                return true;
            }

            UITrackpad.updateAmplifier(context);

            return true;
        }
    }
}
