package mchorse.bbs_mod.ui.utils.bones;

import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.cubic.ModelInstance;
import mchorse.bbs_mod.forms.forms.BodyPart;
import mchorse.bbs_mod.forms.forms.Form;
import mchorse.bbs_mod.forms.forms.ModelForm;
import mchorse.bbs_mod.forms.renderers.ModelFormRenderer;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.ui.framework.elements.input.list.UIStringList;
import mchorse.bbs_mod.utils.StringUtils;
import mchorse.bbs_mod.utils.colors.Colors;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Predicate;

/**
 * Bone list that understands hierarchy. The backing list still holds plain string ids
 * (bone keys or attachment paths), so every {@link UIStringList} caller keeps working;
 * this class only adds per-id display metadata: indentation depth, a short label for
 * the tree view and a full label for search results (where the indentation context is
 * gone, e.g. an attachment bone needs its form's track name to stay recognizable).
 *
 * <p>Two ways to feed it: {@link #fillBones}/{@link #fillAttachments} rebuild the list
 * and the metadata together (the picker), or {@link #setHierarchy} sets the metadata
 * alone while the host keeps managing the list contents itself (the pose editor's
 * bone list, which refills on every search keystroke — the metadata survives because
 * {@code clear()} intentionally does not touch it).</p>
 */
public class UIBoneTreeList extends UIStringList
{
    public static final int INDENT = 6;

    private static final int GUIDE_COLOR = Colors.A12 | 0xFFFFFF;

    private final Map<String, Integer> depths = new HashMap<>();
    private final Map<String, String> treeLabels = new HashMap<>();
    private final Map<String, String> fullLabels = new HashMap<>();

    private Predicate<String> disabled;

    /** Hosts that filter by refilling the list (instead of {@link #filter}) set this
     *  while a query is active, so matches render flat like built-in filtering does. */
    private boolean flat;

    public UIBoneTreeList(Consumer<List<String>> callback)
    {
        super(callback);
    }

    /**
     * Mark ids the host refuses to accept (e.g. IK targets that would close a cycle).
     * They render gray and {@link #isDisabled} lets the picker ignore clicks on them.
     */
    public UIBoneTreeList disabled(Predicate<String> predicate)
    {
        this.disabled = predicate;

        return this;
    }

    public boolean isDisabled(String id)
    {
        return this.disabled != null && id != null && !id.isEmpty() && this.disabled.test(id);
    }

    public void flat(boolean flat)
    {
        this.flat = flat;
    }

    /** First row currently on screen (respecting the search filter) — the Enter pick. */
    public String getFirstVisible()
    {
        return this.getElementAt(0);
    }

    /** Insert a special entry (like "None") above the tree, outside of any hierarchy. */
    public void prepend(String id, String label)
    {
        this.list.add(0, id);
        this.depths.put(id, 0);
        this.treeLabels.put(id, label);
        this.fullLabels.put(id, label);
        this.update();
    }

    private void resetMeta()
    {
        this.depths.clear();
        this.treeLabels.clear();
        this.fullLabels.clear();
    }

    /**
     * Set only the hierarchy metadata from a model, leaving the list contents to the
     * host. Passing a null model clears the metadata (every row renders flat).
     */
    public void setHierarchy(IModel model, Predicate<String> hidden)
    {
        this.resetMeta();

        if (model != null)
        {
            for (String root : model.getRootGroupKeys())
            {
                this.walkBone(model, root, 0, hidden, false);
            }
        }
    }

    /**
     * Fill from a model's bone hierarchy, pre-order, skipping hidden bones. A hidden
     * bone's children stay visible and take over its depth, mirroring how the flat
     * lists used to just remove disabled bones from the hierarchy-ordered key list.
     */
    public void fillBones(IModel model, Collection<String> hidden)
    {
        this.clear();
        this.resetMeta();

        if (model != null)
        {
            Predicate<String> predicate = hidden == null ? null : hidden::contains;

            for (String root : model.getRootGroupKeys())
            {
                this.walkBone(model, root, 0, predicate, true);
            }
        }

        this.update();
    }

    private void walkBone(IModel model, String bone, int depth, Predicate<String> hidden, boolean fill)
    {
        boolean visible = hidden == null || !hidden.test(bone);

        if (visible)
        {
            if (fill)
            {
                this.list.add(bone);
            }

            this.depths.put(bone, depth);
        }

        for (String child : model.getDirectChildrenKeys(bone))
        {
            this.walkBone(model, child, depth + (visible ? 1 : 0), hidden, fill);
        }
    }

    /**
     * Fill with a plain list of bone names, no hierarchy — the fallback for forms
     * whose bones don't come from an {@link IModel} (e.g. mob forms' model parts).
     */
    public void fillFlat(Collection<String> bones)
    {
        this.clear();
        this.resetMeta();

        this.list.addAll(bones);
        this.update();
    }

    /**
     * Fill from a form's attachment keys (see {@code FormRenderer.collectMatrices}):
     * every form in the body part tree becomes a header row, its model bones nest
     * under it in their own hierarchy order. The key set stays the source of truth —
     * only ids present in it are listed, so the picker can never offer an attachment
     * the matrix cache doesn't actually resolve.
     */
    public void fillAttachments(Form form, Collection<String> keys)
    {
        this.clear();
        this.resetMeta();

        Set<String> keySet = new HashSet<>(keys);

        if (form != null)
        {
            this.walkForm(form, "", 0, keySet);
        }

        /* Safety net: keys the static walk didn't reach (exotic renderers) go in flat,
         * so switching from the raw key list to the tree can't lose selectable values. */
        List<String> missed = new ArrayList<>();

        for (String key : keySet)
        {
            if (!this.list.contains(key))
            {
                missed.add(key);
            }
        }

        missed.sort(String::compareToIgnoreCase);
        this.list.addAll(missed);

        this.update();
    }

    private void walkForm(Form form, String path, int depth, Set<String> keys)
    {
        if (keys.contains(path))
        {
            String trackName = form.getTrackName("");
            String label = trackName.isEmpty() ? form.getFormIdOrName() : trackName;

            this.list.add(path);
            this.depths.put(path, depth);
            this.treeLabels.put(path, label);
            this.fullLabels.put(path, label);
        }

        if (form instanceof ModelForm modelForm)
        {
            ModelInstance instance = ModelFormRenderer.getModel(modelForm);

            if (instance != null && instance.model != null)
            {
                for (String root : instance.model.getRootGroupKeys())
                {
                    this.walkFormBone(form, instance.model, root, path, depth + 1, keys);
                }
            }
        }

        int i = 0;

        /* The index must advance for every part, even form-less ones — that is how
         * collectMatrices numbers the paths. */
        for (BodyPart part : form.parts.getAllTyped())
        {
            Form child = part.getForm();

            if (child != null)
            {
                this.walkForm(child, StringUtils.combinePaths(path, String.valueOf(i)), depth + 1, keys);
            }

            i += 1;
        }
    }

    private void walkFormBone(Form owner, IModel model, String bone, String formPath, int depth, Set<String> keys)
    {
        String key = StringUtils.combinePaths(formPath, bone);

        if (keys.contains(key))
        {
            this.list.add(key);
            this.depths.put(key, depth);
            this.treeLabels.put(key, bone);
            this.fullLabels.put(key, owner.getTrackName(key));
        }

        for (String child : model.getDirectChildrenKeys(bone))
        {
            this.walkFormBone(owner, model, child, formPath, depth + 1, keys);
        }
    }

    @Override
    protected String elementToString(UIContext context, int i, String element)
    {
        return this.fullLabels.getOrDefault(element, element);
    }

    @Override
    protected void renderElementPart(UIContext context, String element, int i, int x, int y, boolean hover, boolean selected)
    {
        /* Search results render flat with their full label — indentation without the
         * parent rows above it is just a lie about structure. */
        boolean filtering = this.flat || this.isFiltering();
        int depth = filtering ? 0 : this.depths.getOrDefault(element, 0);
        int h = this.scroll.scrollItemSize;

        for (int level = 1; level <= depth; level++)
        {
            int lx = x + 4 + (level - 1) * INDENT + 2;

            context.batcher.box(lx, y, lx + 1, y + h, GUIDE_COLOR);
        }

        String label = filtering ? this.fullLabels.getOrDefault(element, element) : this.treeLabels.getOrDefault(element, element);
        int color = this.isDisabled(element) ? Colors.GRAY : (hover ? Colors.HIGHLIGHT : Colors.WHITE);

        context.batcher.textShadow(label, x + 4 + depth * INDENT, y + (h - context.batcher.getFont().getHeight()) / 2, color);
    }
}
