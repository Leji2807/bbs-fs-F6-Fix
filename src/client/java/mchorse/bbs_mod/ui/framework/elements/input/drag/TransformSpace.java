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
 * {@link #VIEW} to the camera's right/up/forward. {@link #PARENT} aligns to
 * the frame the bone's channels compose in &mdash; its parent bone (or the
 * model root / world for a top-level bone). The non-local gizmo placement
 * already carries that frame: the matrix cache's origin flavour is the bone's
 * frame BEFORE its own rotation, so PARENT simply keeps the placed axes
 * (see {@code Gizmo.reorientForSpace}); the model block maps it to GLOBAL,
 * its transform composing straight onto the world. The four-way cycle
 * replaces the old local/global boolean, so {@code space == LOCAL} is exactly
 * the former {@code local} flag and every consumer that only distinguished
 * local from not-local keeps working with {@link #isLocal()}.
 */
public enum TransformSpace
{
    /** The bone's own axes — the gizmo and constrained edits follow the pose. */
    LOCAL(true),

    /** The world axes — a constrained edit runs along fixed global X/Y/Z. */
    GLOBAL(true),

    /** The camera's right/up/forward — a constrained edit runs in screen space. */
    VIEW(true),

    /** The parent's frame — the frame the bone's own channels compose in.
     *  Rotation here deliberately bumps the driven channel directly (the
     *  pre-spaces gizmo behaviour): exact single-parameter turns with native
     *  &gt;360° winding, Blender's gimbal-style workflow. */
    PARENT(true);

    /** Whether the frame math is wired up; unimplemented spaces are shown but not selectable. */
    public final boolean implemented;

    TransformSpace(boolean implemented)
    {
        this.implemented = implemented;
    }

    /** Whether this is the local frame; the single distinction older consumers make. */
    public boolean isLocal()
    {
        return this == LOCAL;
    }

    /** The next selectable frame in the cycle, skipping any that aren't implemented yet. */
    public TransformSpace next()
    {
        TransformSpace[] values = values();
        TransformSpace next = this;

        do
        {
            next = values[(next.ordinal() + 1) % values.length];
        }
        while (!next.implemented && next != this);

        return next;
    }
}
