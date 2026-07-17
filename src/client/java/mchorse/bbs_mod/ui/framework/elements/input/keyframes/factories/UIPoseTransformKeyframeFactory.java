package mchorse.bbs_mod.ui.framework.elements.input.keyframes.factories;

import mchorse.bbs_mod.ui.film.replays.UIReplaysEditorUtils;
import mchorse.bbs_mod.ui.Keys;
import mchorse.bbs_mod.ui.UIKeys;
import mchorse.bbs_mod.ui.dashboard.panels.UIDashboardPanels;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.ui.framework.elements.buttons.UIIcon;
import mchorse.bbs_mod.ui.framework.elements.input.UIColor;
import mchorse.bbs_mod.ui.framework.elements.input.UIPropTransform;
import mchorse.bbs_mod.ui.framework.elements.input.UITrackpad;
import mchorse.bbs_mod.ui.utils.UIConstants;
import mchorse.bbs_mod.ui.framework.elements.input.keyframes.UIKeyframeSheet;
import mchorse.bbs_mod.ui.framework.elements.input.keyframes.UIKeyframes;
import mchorse.bbs_mod.ui.utils.UI;
import mchorse.bbs_mod.ui.utils.icons.Icons;
import mchorse.bbs_mod.utils.Direction;
import mchorse.bbs_mod.utils.keyframes.Keyframe;
import mchorse.bbs_mod.utils.pose.PoseTransform;
import mchorse.bbs_mod.utils.pose.Transform;

import java.util.function.Consumer;

public class UIPoseTransformKeyframeFactory extends UIKeyframeFactory<PoseTransform>
{
    public UITrackpad fix;
    public UIColor color;
    /** Lighting on/off as an icon toggle; the panel paints its highlight in {@link #render}. */
    public UIIcon lighting;
    public UIPropTransform transform;

    public UIPoseTransformKeyframeFactory(Keyframe<PoseTransform> keyframe, UIKeyframes editor)
    {
        super(keyframe, editor);

        this.transform = new UIPoseTransforms(this);
        this.transform.enableHotkeys();
        this.transform.setTransform(keyframe.getValue());

        this.keys().register(Keys.TRANSFORMATIONS_TOGGLE_FIX, this::toggleFix).category(UIKeys.TRANSFORMS_KEYS_CATEGORY);

        this.fix = new UITrackpad((v) ->
        {
            if (this.transform.getTransform() instanceof PoseTransform)
            {
                UIPoseTransforms.apply(editor, keyframe, (poseT) -> poseT.fix = v.floatValue());
            }
        });
        this.fix.limit(0D, 1D).increment(1D).values(0.1, 0.05D, 0.2D);
        this.fix.tooltip(UIKeys.POSE_CONTEXT_FIX_TOOLTIP);
        this.fix.setValue(keyframe.getValue().fix);

        this.color = new UIColor((c) ->
        {
            if (this.transform.getTransform() instanceof PoseTransform)
            {
                UIPoseTransforms.apply(editor, keyframe, (poseT) -> poseT.color.set(c));
            }
        }).noLabel();
        this.color.withAlpha();
        this.color.tooltip(UIKeys.POSE_CONTEXT_COLOR);
        this.color.setColor(keyframe.getValue().color.getARGBColor());

        this.lighting = new UIIcon(Icons.LIGHT, (b) ->
        {
            if (this.transform.getTransform() instanceof PoseTransform)
            {
                boolean value = !this.lighting.isActive();

                this.lighting.active(value);
                UIPoseTransforms.apply(editor, keyframe, (poseT) -> poseT.lighting = value ? 0F : 1F);
            }
        });
        this.lighting.wh(UIConstants.CONTROL_HEIGHT, UIConstants.CONTROL_HEIGHT);
        this.lighting.tooltip(UIKeys.FORMS_EDITORS_GENERAL_LIGHTING);
        this.lighting.active(keyframe.getValue().lighting == 0F);

        /* One row, each element named by its tooltip — same layout as the pose
         * editor's, which this panel mirrors. */
        this.scroll.add(UI.row(2, 0, UIConstants.CONTROL_HEIGHT, this.fix, this.color, this.lighting), this.transform.marginTop(4));
    }

    @Override
    public void render(UIContext context)
    {
        if (this.lighting.isActive())
        {
            UIDashboardPanels.renderHighlight(context.batcher, this.lighting.area, Direction.LEFT);
        }

        super.render(context);
    }

    private void toggleFix()
    {
        if (!(this.transform.getTransform() instanceof PoseTransform))
        {
            return;
        }
        float next = this.fix.getValue() >= 0.5F ? 0F : 1F;
        this.fix.setValue(next);
        UIPoseTransforms.apply(this.editor, this.keyframe, (poseT) -> poseT.fix = next);
    }

    public static class UIPoseTransforms extends UIKeyframePropTransform
    {
        private UIPoseTransformKeyframeFactory editor;

        public UIPoseTransforms(UIPoseTransformKeyframeFactory editor)
        {
            this.editor = editor;
        }

        @Override
        protected void applyToSelection(Consumer<Transform> consumer)
        {
            apply(this.editor.editor, this.editor.keyframe, (poseT) -> consumer.accept(poseT));
        }

        @Override
        protected void applyDuringRecording(int tick, Consumer<Transform> consumer)
        {
            applyRecording(this.editor.editor, this.editor.keyframe, tick, (poseT) -> consumer.accept(poseT));
        }

        @Override
        protected Transform getRecordedTransform(int tick)
        {
            UIKeyframeSheet sheet = this.editor.editor.getGraph().getSheet(this.editor.keyframe);
            Keyframe<PoseTransform> recorded = UIReplaysEditorUtils.ensureKeyframe(sheet, tick);

            return recorded == null ? null : recorded.getValue();
        }

        public static void applyRecording(UIKeyframes editor, Keyframe keyframe, int tick, Consumer<PoseTransform> consumer)
        {
            UIReplaysEditorUtils.forEachRecordedKeyframe(editor, keyframe, tick, (recorded) ->
            {
                PoseTransform transform = (PoseTransform) recorded.getValue();

                recorded.preNotify();
                consumer.accept(transform);
                recorded.postNotify();
            });
        }

        public static void apply(UIKeyframes editor, Keyframe keyframe, Consumer<PoseTransform> consumer)
        {
            UIReplaysEditorUtils.forEachSelectedKeyframe(editor, keyframe, (selected) ->
            {
                PoseTransform transform = (PoseTransform) selected.getValue();

                selected.preNotify();
                consumer.accept(transform);
                selected.postNotify();
            });
        }
    }
}
