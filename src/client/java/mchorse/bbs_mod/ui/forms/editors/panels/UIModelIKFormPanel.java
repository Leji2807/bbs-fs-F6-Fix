package mchorse.bbs_mod.ui.forms.editors.panels;

import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.cubic.ModelInstance;
import mchorse.bbs_mod.cubic.ik.ModelIKConfig;
import mchorse.bbs_mod.cubic.ik.ModelIKIO;
import mchorse.bbs_mod.cubic.ik.ModelIKRuntime;
import mchorse.bbs_mod.data.types.MapType;
import mchorse.bbs_mod.forms.forms.ModelForm;
import mchorse.bbs_mod.forms.renderers.ModelFormRenderer;
import mchorse.bbs_mod.l10n.keys.IKey;
import mchorse.bbs_mod.ui.UIKeys;
import mchorse.bbs_mod.ui.forms.editors.forms.UIForm;
import mchorse.bbs_mod.ui.forms.editors.utils.UIDebugOverlayContextMenu;
import mchorse.bbs_mod.ui.framework.elements.UIElement;
import mchorse.bbs_mod.ui.framework.elements.UISection;
import mchorse.bbs_mod.ui.framework.elements.buttons.UIButton;
import mchorse.bbs_mod.ui.framework.elements.buttons.UIIcon;
import mchorse.bbs_mod.ui.framework.elements.buttons.UIToggle;
import mchorse.bbs_mod.ui.framework.elements.input.UISliderTrackpad;
import mchorse.bbs_mod.ui.framework.elements.input.UITrackpad;
import mchorse.bbs_mod.ui.framework.elements.input.list.UIStringList;
import mchorse.bbs_mod.ui.utils.UI;
import mchorse.bbs_mod.ui.utils.UIConstants;
import mchorse.bbs_mod.ui.utils.presets.UIDataContextMenu;
import mchorse.bbs_mod.ui.utils.icons.Icons;
import mchorse.bbs_mod.utils.pose.ModelIKManager;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class UIModelIKFormPanel extends UIFormPanel<ModelForm>
{
    public UIStringList bones;

    public UIToggle debug;
    public UIToggle enabled;
    public UIButton target;
    public UITrackpad chainLength;
    public UIToggle pole;
    public UIButton poleTarget;
    public UISliderTrackpad poleAngle;
    public UISliderTrackpad softness;
    public UISliderTrackpad weight;
    public UIToggle tipRotation;
    public UIToggle classic;

    public UIToggle lockX;
    public UIToggle lockY;
    public UIToggle lockZ;
    public UIToggle limitX;
    public UIToggle limitY;
    public UIToggle limitZ;
    public UISliderTrackpad limitMinX;
    public UISliderTrackpad limitMaxX;
    public UISliderTrackpad limitMinY;
    public UISliderTrackpad limitMaxY;
    public UISliderTrackpad limitMinZ;
    public UISliderTrackpad limitMaxZ;
    public UISliderTrackpad stiffnessX;
    public UISliderTrackpad stiffnessY;
    public UISliderTrackpad stiffnessZ;
    public UIToggle stretch;

    private String selectedBone = "";
    private Map<String, IKData> ikData = new HashMap<>();
    private Map<String, JointData> jointData = new HashMap<>();
    private ModelInstance model;
    private String presetGroup = "";
    private boolean syncingUI;

    private static class IKData
    {
        public String target = "";
        public int chainLength = ModelIKConfig.DEFAULT_CHAIN_LENGTH;
        public boolean pole = true;
        public String poleTarget = ModelIKConfig.DEFAULT_POLE_TARGET;
        public float poleAngle = ModelIKConfig.DEFAULT_POLE_ANGLE;
        public float softness = ModelIKConfig.DEFAULT_SOFTNESS;
        public float weight = ModelIKConfig.DEFAULT_WEIGHT;
        public boolean enabled = true;
        public boolean tipRotation = ModelIKConfig.DEFAULT_TIP_ROTATION;
        public boolean stretch = ModelIKConfig.DEFAULT_STRETCH;
        public boolean classic = ModelIKConfig.DEFAULT_CLASSIC;
    }

    /** Mutable UI shadow of {@link ModelIKConfig.JointDoF} — the selected bone's joint freedom. */
    private static class JointData
    {
        public boolean lockX, lockY, lockZ;
        public boolean limitX, limitY, limitZ;
        public float minX = ModelIKConfig.JointDoF.DEFAULT_MIN;
        public float maxX = ModelIKConfig.JointDoF.DEFAULT_MAX;
        public float minY = ModelIKConfig.JointDoF.DEFAULT_MIN;
        public float maxY = ModelIKConfig.JointDoF.DEFAULT_MAX;
        public float minZ = ModelIKConfig.JointDoF.DEFAULT_MIN;
        public float maxZ = ModelIKConfig.JointDoF.DEFAULT_MAX;
        public float stiffnessX, stiffnessY, stiffnessZ;

        public static JointData from(ModelIKConfig.JointDoF dof)
        {
            JointData data = new JointData();

            data.lockX = dof.lockX();
            data.lockY = dof.lockY();
            data.lockZ = dof.lockZ();
            data.limitX = dof.limitX();
            data.limitY = dof.limitY();
            data.limitZ = dof.limitZ();
            data.minX = dof.minX();
            data.maxX = dof.maxX();
            data.minY = dof.minY();
            data.maxY = dof.maxY();
            data.minZ = dof.minZ();
            data.maxZ = dof.maxZ();
            data.stiffnessX = dof.stiffnessX();
            data.stiffnessY = dof.stiffnessY();
            data.stiffnessZ = dof.stiffnessZ();

            return data;
        }

        public ModelIKConfig.JointDoF toDoF()
        {
            return new ModelIKConfig.JointDoF(this.lockX, this.lockY, this.lockZ,
                this.limitX, this.minX, this.maxX,
                this.limitY, this.minY, this.maxY,
                this.limitZ, this.minZ, this.maxZ,
                this.stiffnessX, this.stiffnessY, this.stiffnessZ);
        }
    }

    public UIModelIKFormPanel(UIForm editor)
    {
        super(editor);

        this.bones = new UIStringList((l) ->
        {
            this.selectedBone = l.isEmpty() ? "" : l.get(0);
            this.updateLabels();
        });
        this.bones.background().h(UIConstants.LIST_ITEM_HEIGHT * 8);
        this.bones.context(() -> new UIDataContextMenu(ModelIKManager.INSTANCE, this.presetGroup, this::toPresetData, this::applyPresetData).tooltips("_CopyModelIK",
            UIKeys.FORMS_EDITORS_MODEL_IK_CONTEXT_COPY,
            UIKeys.FORMS_EDITORS_MODEL_IK_CONTEXT_PASTE,
            UIKeys.FORMS_EDITORS_MODEL_IK_CONTEXT_RESET,
            UIKeys.FORMS_EDITORS_MODEL_IK_CONTEXT_SAVE,
            UIKeys.FORMS_EDITORS_MODEL_IK_CONTEXT_NAME
        ));

        this.debug = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_DEBUG, (b) -> BBSSettings.ikDebug.enabled.set(b.getValue()));
        this.debug.setValue(BBSSettings.ikDebug.enabled.get());
        this.debug.context(() -> new UIDebugOverlayContextMenu(BBSSettings.ikDebug));

        this.enabled = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_ENABLED, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.enabled = b.getValue();
            this.updateLabels();
            this.commitChanges();
        });
        this.enabled.h(UIConstants.CONTROL_HEIGHT);

        this.target = new UIButton(IKey.EMPTY, (b) ->
        {
            if (this.selectedBone.isEmpty()) return;

            IKData data = this.getOrCreateData(this.selectedBone);
            this.openBoneMenu(data.target, (bone) ->
            {
                data.target = bone;
                this.updateLabels();
                this.commitChanges();
            });
        });

        this.chainLength = new UITrackpad((v) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.chainLength = Math.max(0, (int) v.floatValue());
            this.commitChanges();
        });
        this.chainLength.limit(0).integer();
        this.chainLength.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_CHAIN_LENGTH);

        this.pole = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_POLE, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.pole = b.getValue();
            this.updateLabels();
            this.commitChanges();
        });
        this.pole.h(UIConstants.CONTROL_HEIGHT);

        this.poleTarget = new UIButton(IKey.EMPTY, (b) ->
        {
            if (this.selectedBone.isEmpty()) return;

            IKData data = this.getOrCreateData(this.selectedBone);
            this.openBoneMenu(data.poleTarget, (bone) ->
            {
                data.poleTarget = bone;
                this.updateLabels();
                this.commitChanges();
            });
        });

        this.poleAngle = new UISliderTrackpad((v) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.poleAngle = v.floatValue();
            this.commitChanges();
        });
        this.poleAngle.limit(-180D, 180D).increment(5D).values(1D, 0.5D, 5D);
        this.poleAngle.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_POLE_ANGLE);

        this.softness = new UISliderTrackpad((v) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.softness = v.floatValue();
            this.commitChanges();
        });
        this.softness.limit(0D, 1D).increment(0.05D).values(0.05D, 0.01D, 0.1D);
        this.softness.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_SOFTNESS);

        this.weight = new UISliderTrackpad((v) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.weight = v.floatValue();
            this.commitChanges();
        });
        this.weight.limit(0D, 1D).increment(0.1D).values(0.1D, 0.05D, 0.2D);
        this.weight.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_WEIGHT);

        this.tipRotation = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_TIP_ROTATION, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.tipRotation = b.getValue();
            this.commitChanges();
        });

        this.stretch = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_STRETCH, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.stretch = b.getValue();
            this.commitChanges();
        });

        this.classic = new UIToggle(UIKeys.FORMS_EDITORS_MODEL_IK_CLASSIC, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            IKData data = this.getOrCreateData(this.selectedBone);
            data.classic = b.getValue();
            this.updateLabels();
            this.commitChanges();
        });
        this.classic.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_CLASSIC_TOOLTIP);

        UISection settings = new UISection(UIKeys.FORMS_EDITORS_MODEL_IK_SETTINGS);

        /* Blender's IK constraint order: target, then the pole group, chain
         * length, the tip/stretch toggles, and influence last (Blender puts the
         * constraint's Influence slider at the very bottom). enabled/pole/softness
         * stay — they are ours; softness rides just above influence as the other
         * solve tuning scalar. enabled+target and pole+poleTarget each pair into
         * one labelRow — the toggle names itself in the label slot, the bone picker
         * pins to the shared value column (same grid as the pose editor's
         * lighting+colour row). */
        settings.fields.add(
            UI.labelRow(this.enabled, this.target),
            UI.labelRow(this.pole, this.poleTarget),
            UI.labelRow(UIKeys.FORMS_EDITORS_MODEL_IK_POLE_ANGLE, this.poleAngle),
            UI.labelRow(UIKeys.FORMS_EDITORS_MODEL_IK_CHAIN_LENGTH, this.chainLength),
            this.tipRotation,
            this.stretch,
            this.classic,
            UI.labelRow(UIKeys.FORMS_EDITORS_MODEL_IK_SOFTNESS, this.softness),
            UI.labelRow(UIKeys.FORMS_EDITORS_MODEL_IK_WEIGHT, this.weight)
        );

        /* The selected bone's JOINT freedom — per axis: lock, limit (degrees), stiffness.
         * Per BONE, not per chain: a bone shared by several chains has one set of joints. */
        this.lockX = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LOCK.format("X"), (d, v) -> d.lockX = v);
        this.lockY = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LOCK.format("Y"), (d, v) -> d.lockY = v);
        this.lockZ = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LOCK.format("Z"), (d, v) -> d.lockZ = v);

        this.limitX = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LIMIT.format("X"), (d, v) -> d.limitX = v);
        this.limitY = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LIMIT.format("Y"), (d, v) -> d.limitY = v);
        this.limitZ = this.jointToggle(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_LIMIT.format("Z"), (d, v) -> d.limitZ = v);

        this.limitMinX = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MIN, (d, v) -> d.minX = v);
        this.limitMaxX = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MAX, (d, v) -> d.maxX = v);
        this.limitMinY = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MIN, (d, v) -> d.minY = v);
        this.limitMaxY = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MAX, (d, v) -> d.maxY = v);
        this.limitMinZ = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MIN, (d, v) -> d.minZ = v);
        this.limitMaxZ = this.jointDegrees(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_MAX, (d, v) -> d.maxZ = v);

        this.stiffnessX = this.jointStiffness((d, v) -> d.stiffnessX = v);
        this.stiffnessY = this.jointStiffness((d, v) -> d.stiffnessY = v);
        this.stiffnessZ = this.jointStiffness((d, v) -> d.stiffnessZ = v);

        UISection joint = new UISection(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT);

        joint.fields.add(
            this.lockX, this.lockY, this.lockZ,
            this.limitX.marginTop(UIConstants.SECTION_GAP), UI.row(this.limitMinX, this.limitMaxX),
            this.limitY.marginTop(UIConstants.SECTION_GAP), UI.row(this.limitMinY, this.limitMaxY),
            this.limitZ.marginTop(UIConstants.SECTION_GAP), UI.row(this.limitMinZ, this.limitMaxZ),
            UI.label(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_STIFFNESS).marginTop(UIConstants.SECTION_GAP),
            UI.row(this.stiffnessX, this.stiffnessY, this.stiffnessZ)
        );

        UIIcon debugSettings = new UIIcon(Icons.GEAR, (b) -> this.getContext().replaceContextMenu(new UIDebugOverlayContextMenu(BBSSettings.ikDebug)));

        debugSettings.tooltip(UIKeys.MODEL_DEBUG_CONFIGURE);
        debugSettings.wh(20, 14);

        UIElement debugRow = new UIElement();

        debugRow.row(0).preferred(0).height(14);
        debugRow.add(this.debug, debugSettings);

        this.options.add(
            debugRow,
            this.bones,
            settings,
            joint
        );
    }

    private UIToggle jointToggle(IKey label, BiConsumer<JointData, Boolean> setter)
    {
        return new UIToggle(label, (b) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            setter.accept(this.getOrCreateJoint(this.selectedBone), b.getValue());
            this.updateLabels();
            this.commitChanges();
        });
    }

    private UISliderTrackpad jointDegrees(IKey tooltip, BiConsumer<JointData, Float> setter)
    {
        UISliderTrackpad pad = new UISliderTrackpad(this.jointCallback(setter));

        pad.limit(-180D, 180D).increment(5D).values(1D, 0.5D, 5D);
        pad.tooltip(tooltip);

        return pad;
    }

    private UISliderTrackpad jointStiffness(BiConsumer<JointData, Float> setter)
    {
        UISliderTrackpad pad = new UISliderTrackpad(this.jointCallback(setter));

        pad.limit(0D, 1D).increment(0.1D).values(0.1D, 0.05D, 0.2D);
        pad.tooltip(UIKeys.FORMS_EDITORS_MODEL_IK_JOINT_STIFFNESS);

        return pad;
    }

    private Consumer<Double> jointCallback(BiConsumer<JointData, Float> setter)
    {
        return (v) ->
        {
            if (this.syncingUI || this.selectedBone.isEmpty())
            {
                return;
            }

            setter.accept(this.getOrCreateJoint(this.selectedBone), v.floatValue());
            this.commitChanges();
        };
    }

    private JointData getOrCreateJoint(String bone)
    {
        return this.jointData.computeIfAbsent(bone, k -> new JointData());
    }

    @Override
    public void startEdit(ModelForm form)
    {
        super.startEdit(form);

        this.debug.setValue(BBSSettings.ikDebug.enabled.get());

        ModelInstance model = ModelFormRenderer.getModel(form);
        this.model = model;
        this.presetGroup = this.resolvePresetGroup(form, model);

        if (model == null || model.model == null)
        {
            this.bones.setList(Collections.emptyList());
            this.bones.deselect();
            this.selectedBone = "";
            this.ikData.clear();
            this.jointData.clear();

            this.setElementsEnabled(false);
        }
        else
        {
            List<String> bones = new ArrayList<>(model.model.getGroupKeysInHierarchyOrder());
            bones.removeIf(model.getDisabledBones()::contains);

            this.bones.setList(bones);
            this.setElementsEnabled(true);

            this.load();
        }

        this.updateLabels();
        this.options.resize();
    }

    private void setElementsEnabled(boolean enabled)
    {
        this.bones.setEnabled(enabled);
        this.enabled.setEnabled(enabled);
        this.target.setEnabled(enabled);
        this.chainLength.setEnabled(enabled);
        this.pole.setEnabled(enabled);
        this.poleTarget.setEnabled(enabled);
        this.poleAngle.setEnabled(enabled);
        this.softness.setEnabled(enabled);
        this.weight.setEnabled(enabled);
        this.tipRotation.setEnabled(enabled);
        this.stretch.setEnabled(enabled);
        this.classic.setEnabled(enabled);
        this.setJointEnabled(enabled);
    }

    private void setJointEnabled(boolean enabled)
    {
        this.lockX.setEnabled(enabled);
        this.lockY.setEnabled(enabled);
        this.lockZ.setEnabled(enabled);
        this.limitX.setEnabled(enabled);
        this.limitY.setEnabled(enabled);
        this.limitZ.setEnabled(enabled);
        this.limitMinX.setEnabled(enabled);
        this.limitMaxX.setEnabled(enabled);
        this.limitMinY.setEnabled(enabled);
        this.limitMaxY.setEnabled(enabled);
        this.limitMinZ.setEnabled(enabled);
        this.limitMaxZ.setEnabled(enabled);
        this.stiffnessX.setEnabled(enabled);
        this.stiffnessY.setEnabled(enabled);
        this.stiffnessZ.setEnabled(enabled);
    }

    @Override
    public boolean pickBoneInList(String bone)
    {
        if (bone == null || bone.isEmpty() || !this.bones.getList().contains(bone))
        {
            return false;
        }

        this.selectedBone = bone;
        this.bones.setCurrentScroll(bone);
        this.updateLabels();

        return true;
    }

    private void openBoneMenu(String current, Consumer<String> callback)
    {
        if (this.bones.getList().isEmpty())
        {
            return;
        }

        this.getContext().replaceContextMenu((menu) ->
        {
            boolean none = current == null || current.isEmpty();

            menu.action(Icons.REMOVE, UIKeys.GENERAL_NONE, none, () -> callback.accept(""));

            for (String bone : this.bones.getList())
            {
                boolean selected = bone.equals(current);

                menu.action(Icons.LIMB, IKey.constant(bone), selected, () -> callback.accept(bone));
            }
        });
    }

    private void updateLabels()
    {
        if (this.target == null || this.enabled == null)
        {
            return;
        }

        IKData data = this.ikData.get(this.selectedBone);
        JointData joint = this.jointData.get(this.selectedBone);

        String targetLabel = data == null ? "" : data.target;
        boolean active = data != null && data.enabled;
        boolean poleOn = data != null && data.pole;
        boolean canEdit = !this.selectedBone.isEmpty() && this.bones.isEnabled() && active;

        /* Cycle validation, but the two cases differ. A TARGET the chain itself drives
         * closes a feedback loop and the chain does NOT compile — loud "(CYCLE!)".
         * A POLE on a chain bone is not fatal: the compiler quietly drops it and the
         * chain solves with the rest-side auto pole instead, so it gets a softer
         * "on chain → auto pole" hint, not the does-not-compile marker. */
        boolean cyclicTarget = data != null && this.isCyclic(data, targetLabel);
        boolean cyclicPole = data != null && this.isCyclic(data, data.poleTarget);

        this.syncingUI = true;

        try
        {
            this.target.label = UIKeys.FORMS_EDITORS_MODEL_IK_TARGET.format(this.formatBone(targetLabel) + (cyclicTarget ? UIKeys.FORMS_EDITORS_MODEL_IK_CYCLE.get() : ""));
            this.chainLength.setValue(data == null ? ModelIKConfig.DEFAULT_CHAIN_LENGTH : data.chainLength);
            this.pole.setValue(poleOn);
            this.poleTarget.label = UIKeys.FORMS_EDITORS_MODEL_IK_POLE_TARGET.format(this.formatBone(data == null ? "" : data.poleTarget) + (cyclicPole ? UIKeys.FORMS_EDITORS_MODEL_IK_POLE_CYCLE.get() : ""));
            this.poleAngle.setValue(data == null ? ModelIKConfig.DEFAULT_POLE_ANGLE : data.poleAngle);
            this.softness.setValue(data == null ? ModelIKConfig.DEFAULT_SOFTNESS : data.softness);
            this.weight.setValue(data == null ? ModelIKConfig.DEFAULT_WEIGHT : data.weight);
            this.tipRotation.setValue(data != null && data.tipRotation);
            this.stretch.setValue(data != null && data.stretch);
            this.classic.setValue(data != null && data.classic);

            /* The classic toggle is loud about its fallback: a classic chain that
             * is not exactly two bones, or shares a bone with another enabled
             * chain, solves on the core instead — the label says so right where
             * the box was ticked, no runtime surprise. */
            boolean classicFallsBack = data != null && data.classic && this.classicFallsBack(data);

            this.classic.label = classicFallsBack ? UIKeys.FORMS_EDITORS_MODEL_IK_CLASSIC_FALLBACK : UIKeys.FORMS_EDITORS_MODEL_IK_CLASSIC;
            this.enabled.setEnabled(this.bones.isEnabled() && !this.selectedBone.isEmpty());
            this.enabled.setValue(active);

            this.lockX.setValue(joint != null && joint.lockX);
            this.lockY.setValue(joint != null && joint.lockY);
            this.lockZ.setValue(joint != null && joint.lockZ);
            this.limitX.setValue(joint != null && joint.limitX);
            this.limitY.setValue(joint != null && joint.limitY);
            this.limitZ.setValue(joint != null && joint.limitZ);
            this.limitMinX.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MIN : joint.minX);
            this.limitMaxX.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MAX : joint.maxX);
            this.limitMinY.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MIN : joint.minY);
            this.limitMaxY.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MAX : joint.maxY);
            this.limitMinZ.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MIN : joint.minZ);
            this.limitMaxZ.setValue(joint == null ? ModelIKConfig.JointDoF.DEFAULT_MAX : joint.maxZ);
            this.stiffnessX.setValue(joint == null ? 0D : joint.stiffnessX);
            this.stiffnessY.setValue(joint == null ? 0D : joint.stiffnessY);
            this.stiffnessZ.setValue(joint == null ? 0D : joint.stiffnessZ);
        }
        finally
        {
            this.syncingUI = false;
        }

        /* The joint is a property of the BONE, editable regardless of whether a chain
         * ends here — it affects every chain running through this bone. */
        boolean canEditJoint = !this.selectedBone.isEmpty() && this.bones.isEnabled();

        this.setJointEnabled(canEditJoint);
        this.limitMinX.setEnabled(canEditJoint && joint != null && joint.limitX);
        this.limitMaxX.setEnabled(canEditJoint && joint != null && joint.limitX);
        this.limitMinY.setEnabled(canEditJoint && joint != null && joint.limitY);
        this.limitMaxY.setEnabled(canEditJoint && joint != null && joint.limitY);
        this.limitMinZ.setEnabled(canEditJoint && joint != null && joint.limitZ);
        this.limitMaxZ.setEnabled(canEditJoint && joint != null && joint.limitZ);

        this.target.setEnabled(canEdit);
        this.chainLength.setEnabled(canEdit);
        this.pole.setEnabled(canEdit);
        this.poleTarget.setEnabled(canEdit && poleOn);
        this.poleAngle.setEnabled(canEdit && poleOn);
        this.softness.setEnabled(canEdit);
        this.weight.setEnabled(canEdit);
        this.tipRotation.setEnabled(canEdit);
        this.stretch.setEnabled(canEdit);
        this.classic.setEnabled(canEdit);
    }

    /**
     * Whether the selected bone's classic-marked chain would actually solve on
     * the core: wrong shape (not exactly two directed bones) or a bone shared
     * with another enabled chain (overlapping chains merge into one core tree).
     * Mirrors the applier's routing, computed statically from the config.
     */
    private boolean classicFallsBack(IKData data)
    {
        IModel model = this.model == null ? null : this.model.model;

        if (model == null)
        {
            return false;
        }

        if (!ModelIKRuntime.isClassicShape(model, this.selectedBone, data.chainLength, data.tipRotation))
        {
            return true;
        }

        List<String> mine = ModelIKRuntime.chainBones(model, this.selectedBone, data.chainLength);

        for (Map.Entry<String, IKData> entry : this.ikData.entrySet())
        {
            String tip = entry.getKey();
            IKData other = entry.getValue();

            if (tip.equals(this.selectedBone) || other == null || !other.enabled || other.target == null || other.target.isEmpty())
            {
                continue;
            }

            for (String bone : ModelIKRuntime.chainBones(model, tip, other.chainLength))
            {
                if (mine.contains(bone))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private IKData getOrCreateData(String bone)
    {
        return this.ikData.computeIfAbsent(bone, k -> new IKData());
    }

    private String formatBone(String bone)
    {
        return bone == null || bone.isEmpty() ? "-" : bone;
    }

    /** Whether pointing the selected bone's chain at {@code bone} would close a feedback loop. */
    private boolean isCyclic(IKData data, String bone)
    {
        if (bone == null || bone.isEmpty() || this.model == null || this.model.model == null)
        {
            return false;
        }

        return ModelIKRuntime.isCyclicTarget(this.model.model, this.selectedBone, data.chainLength, bone);
    }

    private void load()
    {
        ModelIKConfig config = null;
        if (this.form != null && this.form.ik.get() instanceof MapType map)
        {
            config = ModelIKIO.fromData(map);
        }

        this.load(config);
    }

    private void load(ModelIKConfig config)
    {
        this.ikData.clear();
        this.jointData.clear();

        if (config == null)
        {
            return;
        }

        List<String> bones = this.bones.getList();
        boolean filterByBones = bones != null && !bones.isEmpty();

        if (config.chains() != null)
        {
            for (ModelIKConfig.Chain chain : config.chains())
            {
                if (chain == null || chain.tip() == null || chain.tip().isEmpty())
                {
                    continue;
                }

                if (filterByBones && !bones.contains(chain.tip()))
                {
                    continue;
                }

                IKData data = new IKData();
                data.target = chain.target();
                data.chainLength = chain.chainLength();
                data.pole = chain.pole();
                data.poleTarget = chain.poleTarget();
                data.poleAngle = chain.poleAngle();
                data.softness = chain.softness();
                data.weight = chain.weight();
                data.enabled = chain.enabled();
                data.tipRotation = chain.tipRotation();
                data.stretch = chain.stretch();
                data.classic = chain.classic();
                this.ikData.put(chain.tip(), data);
            }
        }

        for (Map.Entry<String, ModelIKConfig.JointDoF> entry : config.bones().entrySet())
        {
            String bone = entry.getKey();

            if (bone == null || bone.isEmpty() || entry.getValue() == null)
            {
                continue;
            }

            if (filterByBones && !bones.contains(bone))
            {
                continue;
            }

            this.jointData.put(bone, JointData.from(entry.getValue()));
        }
    }

    private MapType toPresetData()
    {
        List<String> bones = this.bones.getList();
        boolean filterByBones = bones != null && !bones.isEmpty();
        List<ModelIKConfig.Chain> out = new ArrayList<>();

        for (Map.Entry<String, IKData> entry : this.ikData.entrySet())
        {
            String tip = entry.getKey();
            IKData data = entry.getValue();

            if (tip == null || tip.isEmpty() || data == null)
            {
                continue;
            }

            if (data.target == null || data.target.isEmpty())
            {
                continue;
            }

            if (filterByBones && (!bones.contains(tip) || !bones.contains(data.target)))
            {
                continue;
            }

            out.add(new ModelIKConfig.Chain(tip, data.target, data.chainLength, data.pole, data.poleTarget, data.poleAngle, data.softness, data.weight, data.enabled, data.tipRotation, data.stretch, data.classic));
        }

        Map<String, ModelIKConfig.JointDoF> joints = new HashMap<>();

        for (Map.Entry<String, JointData> entry : this.jointData.entrySet())
        {
            String bone = entry.getKey();
            JointData data = entry.getValue();

            if (bone == null || bone.isEmpty() || data == null)
            {
                continue;
            }

            if (filterByBones && !bones.contains(bone))
            {
                continue;
            }

            ModelIKConfig.JointDoF dof = data.toDoF();

            if (!dof.isFree())
            {
                joints.put(bone, dof);
            }
        }

        if (out.isEmpty() && joints.isEmpty())
        {
            return new MapType();
        }

        return ModelIKIO.toData(new ModelIKConfig(out, joints));
    }

    private void applyPresetData(MapType map)
    {
        String current = this.selectedBone;

        this.load(ModelIKIO.fromData(map));

        if (current == null || current.isEmpty() || !this.bones.getList().contains(current))
        {
            current = this.bones.getList().isEmpty() ? "" : this.bones.getList().get(0);
        }

        this.selectedBone = current;

        if (current.isEmpty())
        {
            this.bones.deselect();
        }
        else
        {
            this.bones.setCurrentScroll(current);
        }

        this.updateLabels();
        this.commitChanges();
    }

    private void commitChanges()
    {
        if (this.form == null)
        {
            return;
        }

        MapType map = this.toPresetData();
        this.form.ik.set(map.isEmpty() ? null : map);
    }

    private String resolvePresetGroup(ModelForm form, ModelInstance model)
    {
        String group = model != null ? model.getPoseGroup() : "";

        if (group == null || group.isEmpty())
        {
            group = form == null ? "" : form.model.get();
        }

        return group == null ? "" : group;
    }
}
