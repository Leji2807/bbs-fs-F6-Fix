package mchorse.bbs_mod.ui.framework.elements.layout;

import mchorse.bbs_mod.settings.values.ui.EditorLayoutNode;

import java.util.Set;

/**
 * Pluggable storage for a {@link UIDockLayout}. Decouples the docking component from any
 * particular settings value: the Film editor backs this with its per-editor film layout,
 * the Particle editor with its own particle layout, both living in {@code ValueEditorLayout}.
 *
 * <p>The tree is the whole state &mdash; splitter ratios included &mdash; so every edit, down to a
 * splitter drag, arrives through {@link #setRoot}, which is also where the implementation batches
 * the write into its settings value.
 */
public interface ILayoutSource
{
    EditorLayoutNode getRoot();

    void setRoot(EditorLayoutNode root);

    EditorLayoutNode getDefault();

    /** Panels the user hid; the dock's ensure pass must not re-add them. Returns a copy. */
    Set<String> getHiddenPanels();

    void setHiddenPanels(Set<String> hidden);
}
