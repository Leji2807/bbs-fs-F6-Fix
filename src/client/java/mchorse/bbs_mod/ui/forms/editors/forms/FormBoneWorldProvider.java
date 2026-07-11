package mchorse.bbs_mod.ui.forms.editors.forms;

import mchorse.bbs_mod.forms.FormUtils;
import mchorse.bbs_mod.ui.forms.editors.UIFormEditor;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.ui.utils.IWorldTransformProvider;
import mchorse.bbs_mod.utils.StringUtils;
import mchorse.bbs_mod.utils.joml.Matrices;
import org.joml.Matrix4f;

/**
 * World-transform source for the model form editor's pose editor. There is no scene here, so "world"
 * is the form preview's own space — stable, since the preview root doesn't move. It hands back the
 * selected pose bone's full matrix in that space via {@link UIModelForm#getOriginMatrix}, the same
 * sample the pose gizmo drag uses; because {@code ModelFormRenderer.collectMatrices} re-applies the
 * live {@code form.pose} when it builds the matrices, a nudge to the pose shows up in the next sample
 * (what the world paste's finite differences rely on).
 */
public class FormBoneWorldProvider implements IWorldTransformProvider
{
    private final UIModelForm form;

    public FormBoneWorldProvider(UIModelForm form)
    {
        this.form = form;
    }

    @Override
    public boolean getWorldMatrix(Matrix4f out)
    {
        UIFormEditor editor = this.form.editor;
        String bone = this.form.modelPanel.poseEditor.getGroup();

        if (editor == null || bone == null || bone.isEmpty())
        {
            return false;
        }

        Matrix4f matrix = this.form.getOriginMatrix(editor.getSamplingTick());

        if (matrix == null)
        {
            return false;
        }

        out.set(matrix);

        return true;
    }

    /** Same sample for an arbitrary pose bone, keyed the way the matrix cache keys bones. */
    @Override
    public boolean getWorldMatrix(String bone, Matrix4f out)
    {
        UIFormEditor editor = this.form.editor;

        if (editor == null || bone == null || bone.isEmpty())
        {
            return false;
        }

        /* Sample at the gizmo drag's frame, not getSamplingTick(): the multi-bone
         * pivot rotates each bone by the WORLD delta the gizmo drag produced, and
         * that drag froze its axes at the click's partial tick (UIFormEditor.
         * buildGizmoDrag uses context.getTransition()). getSamplingTick() adds the
         * playback cursor, so a PLAYING action would be sampled a sub-frame off the
         * delta — the parent frames disagree and every bone (the active one too)
         * turns slightly wrong. A static action samples identically either way. */
        UIContext context = editor.getContext();
        float transition = context == null ? editor.getSamplingTick() : context.getTransition();
        Matrix4f matrix = this.form.getOrigin(transition, StringUtils.combinePaths(FormUtils.getPath(this.form.form), bone), true);

        /* getOrigin falls back to the shared EMPTY matrix when the bone isn't in
         * the matrix cache; for a multi-bone capture that must read as "can't
         * sample" instead of an identity-placed bone skewing the pivot. */
        if (matrix == null || matrix == Matrices.EMPTY_4F)
        {
            return false;
        }

        out.set(matrix);

        return true;
    }
}
