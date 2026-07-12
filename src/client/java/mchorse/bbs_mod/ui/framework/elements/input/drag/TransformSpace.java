package mchorse.bbs_mod.ui.framework.elements.input.drag;

/**
 * The reference frame a gizmo edit operates in &mdash; Blender's transform
 * orientation, reduced to the frames that make sense for a per-bone editor.
 * It drives both which axes an X/Y/Z-constrained drag turns/slides along and,
 * for the gizmo, which frame its handles are drawn in.
 *
 * <p>{@link #LOCAL} is the historical behaviour: the gizmo aligns to the bone's
 * own axes and a constrained edit runs along them (the panel also switches to
 * relative local nudges here). {@link #GLOBAL} aligns to the world axes and
 * {@link #VIEW} to the camera's right/up/forward. The three-way cycle replaces
 * the old local/global boolean, so {@code space == LOCAL} is exactly the former
 * {@code local} flag and every consumer that only distinguished local from
 * not-local keeps working with {@link #isLocal()}.
 */
public enum TransformSpace
{
    /** The bone's own axes — the gizmo and constrained edits follow the pose. */
    LOCAL,

    /** The world axes — a constrained edit runs along fixed global X/Y/Z. */
    GLOBAL,

    /** The camera's right/up/forward — a constrained edit runs in screen space. */
    VIEW;

    /** Whether this is the local frame; the single distinction older consumers make. */
    public boolean isLocal()
    {
        return this == LOCAL;
    }

    public TransformSpace next()
    {
        TransformSpace[] values = values();

        return values[(this.ordinal() + 1) % values.length];
    }
}
