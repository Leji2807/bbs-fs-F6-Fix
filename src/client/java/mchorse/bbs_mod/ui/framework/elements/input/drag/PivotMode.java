package mchorse.bbs_mod.ui.framework.elements.input.drag;

import mchorse.bbs_mod.BBSSettings;

/**
 * What a multi-bone edit pivots around (Blender's pivot point, reduced to the
 * modes that make sense for bones).
 *
 * <p>{@link #INDIVIDUAL} is the historical behaviour and the default: every
 * edit is a per-channel delta fanned onto each selected bone, so each bone
 * turns about its own origin along its own channel axes. It is what the
 * mirror-edit and alternate-invert workflows compose with, which is why it
 * stays the default. {@link #MEDIAN} and {@link #ACTIVE} rotate/translate the
 * selection as one rigid group about a common point through a
 * {@link SelectionPivotSession}.
 */
public enum PivotMode
{
    /** Per-channel delta onto every bone — each about its own origin. */
    INDIVIDUAL,

    /** Common pivot at the mean of the selected bones' world origins. */
    MEDIAN,

    /** Common pivot at the active (primary) bone's world origin. */
    ACTIVE;

    public static PivotMode current()
    {
        int index = BBSSettings.pivotMode == null ? 0 : BBSSettings.pivotMode.get();
        PivotMode[] values = values();

        return values[Math.floorMod(index, values.length)];
    }

    public PivotMode next()
    {
        PivotMode[] values = values();

        return values[(this.ordinal() + 1) % values.length];
    }
}
