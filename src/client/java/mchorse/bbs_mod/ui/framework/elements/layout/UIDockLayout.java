package mchorse.bbs_mod.ui.framework.elements.layout;

import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.settings.values.ui.EditorLayoutNode;
import mchorse.bbs_mod.ui.dashboard.panels.UIDashboardPanels;
import mchorse.bbs_mod.ui.framework.UIContext;
import mchorse.bbs_mod.ui.framework.elements.UIElement;
import mchorse.bbs_mod.ui.framework.elements.utils.UIDraggable;
import mchorse.bbs_mod.ui.framework.elements.utils.UIRenderable;
import mchorse.bbs_mod.ui.utils.Area;
import mchorse.bbs_mod.ui.utils.icons.Icon;
import mchorse.bbs_mod.ui.utils.icons.Icons;
import mchorse.bbs_mod.utils.Direction;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.colors.Colors;

import org.joml.Vector2i;
import org.lwjgl.glfw.GLFW;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Reusable dockable-panel layout. Owns a set of registered panels and arranges them per an
 * {@link EditorLayoutNode} tree provided by an {@link ILayoutSource}: resizable splitters,
 * drag-to-dock with edge/center drop zones, tab/stack grouping, lock toggle and reset.
 *
 * <p>Panels are registered with {@link #addPanel} and become direct children of this element.
 * Film- or particle-specific behavior (which panels exist, default tree, frameless preview,
 * data gating, follow-up visibility) is supplied as configuration so a single implementation
 * serves both editors.
 */
public class UIDockLayout extends UIElement
{
    /** The drag strip's offset plus its height is the space a panel's content must leave at the top. */
    private static final int DRAG_HANDLE_TOP_OFFSET_PX = 6;
    private static final int DRAG_HANDLE_HEIGHT_PX = 14;
    private static final int SPLITTER_HANDLE_PX = 14;
    private static final int SPLITTER_HANDLE_LINE_PX = 1;
    private static final int SPLITTER_LINK_HITBOX_PADDING_PX = 8;
    private static final int DROP_ZONE_CENTER = -1;
    private static final float DROP_EDGE_MARGIN = 0.2F;
    private static final int DOCK_STACK_TABS_HEIGHT_PX = 20;
    private static final int PANEL_GAP_PX = 4;
    private static final float PANEL_EDGE_EPS = 0.001F;
    /** Normalized handle thickness, so horizontal and vertical handles have comparable grab size. */
    private static final float SPLITTER_HANDLE_THICKNESS_NORM = 0.02F;
    /** Shared zero gutter for the frameless panel, which is flush with its slot. */
    private static final int[] NO_GUTTER = new int[4];

    private final Map<String, UIDockSlot> slotById = new LinkedHashMap<>();
    private final Map<String, Icon> iconById = new HashMap<>();
    private final List<UIDraggable> splitterHandles = new ArrayList<>();
    private final List<SplitterHandleInfo> splitterHandleInfos = new ArrayList<>();
    private final List<UIDockStackTabs> dockStackTabs = new ArrayList<>();
    private final Map<String, DockStackInfo> dockStackByPanelId = new HashMap<>();
    private final List<Integer> draggedSplitterIndices = new ArrayList<>();

    private final UIRenderable canvas = new UIRenderable(this::renderCanvas);
    private final UIRenderable dropHighlight = new UIRenderable(this::renderDropZoneHighlight);

    private boolean layoutLocked = true;
    private String draggingPanelId;
    private String dropTargetPanelId;
    private int dropTargetZone = DROP_ZONE_CENTER;

    /* Configuration */
    private ILayoutSource source;
    private String framelessPanelId;
    private Supplier<Boolean> gate = () -> true;
    private Runnable onChanged = () -> {};
    private Runnable onSplitterDragEnd = () -> {};
    private UnaryOperator<EditorLayoutNode> ensureFn = UnaryOperator.identity();

    /* Configuration setters */

    public UIDockLayout source(ILayoutSource source)
    {
        this.source = source;

        return this;
    }

    /** Panel id whose surface/borders/gutter are skipped (e.g. a frameless 3D preview viewport). */
    public UIDockLayout frameless(String panelId)
    {
        this.framelessPanelId = panelId;

        return this;
    }

    public UIDockLayout gate(Supplier<Boolean> gate)
    {
        this.gate = gate;

        return this;
    }

    /** Run after every layout rebuild so the host can re-sync its own visibility. */
    public UIDockLayout onChanged(Runnable onChanged)
    {
        this.onChanged = onChanged;

        return this;
    }

    public UIDockLayout onSplitterDragEnd(Runnable onSplitterDragEnd)
    {
        this.onSplitterDragEnd = onSplitterDragEnd;

        return this;
    }

    /**
     * Preferred placement for panels missing from a loaded tree. Runs before the generic backstop,
     * which appends whatever the hook did not place, so a hook only has to describe the cases it
     * cares about.
     */
    public UIDockLayout ensure(UnaryOperator<EditorLayoutNode> ensureFn)
    {
        this.ensureFn = ensureFn;

        return this;
    }

    /**
     * Register a panel. The panel becomes a direct child of this element and is arranged by the
     * layout. Call {@link #mount()} once after registering all panels.
     */
    public UIDockLayout addPanel(String id, UIElement panel, Icon icon)
    {
        this.iconById.put(id, icon == null ? Icons.FILE : icon);
        this.slotById.put(id, new UIDockSlot(this, id, panel));

        return this;
    }

    /** Add all children in z-order and run the first layout pass. Call after {@link #addPanel}s. */
    public void mount()
    {
        this.add(this.canvas);

        for (UIDockSlot slot : this.slotById.values())
        {
            this.add(slot);
        }

        this.add(this.dropHighlight);
        this.setupFlex(false);
    }

    /**
     * Space the drag strip occupies at the top of an unlocked panel. Panel content that would sit
     * underneath it has to be pushed down by this much.
     */
    public static int dragStripHeightPx()
    {
        return DRAG_HANDLE_TOP_OFFSET_PX + DRAG_HANDLE_HEIGHT_PX;
    }

    public UIElement getPanel(String id)
    {
        UIDockSlot slot = this.slotById.get(id);

        return slot == null ? null : slot.panel;
    }

    public boolean isLocked()
    {
        return this.layoutLocked;
    }

    public boolean isPanelActive(String panelId)
    {
        DockStackInfo stack = this.dockStackByPanelId.get(panelId);

        return stack != null && panelId.equals(stack.activePanelId);
    }

    public boolean isAnySplitterDragging()
    {
        for (UIDraggable handle : this.splitterHandles)
        {
            if (handle.isDragging())
            {
                return true;
            }
        }

        return false;
    }

    /** Re-apply panel/handle/tab visibility, e.g. after the host's gate condition changed. */
    public void refreshVisibility()
    {
        this.updateTabVisibility();
    }

    private Icon getDockPanelIcon(String panelId)
    {
        return this.iconById.getOrDefault(panelId, Icons.FILE);
    }

    /* Layout settings access */

    private EditorLayoutNode layoutRoot()
    {
        return this.source.getRoot();
    }

    private void setLayoutRoot(EditorLayoutNode root)
    {
        this.source.setRoot(root);
    }

    /* Public actions */

    public void refresh()
    {
        this.clearPanelDragState();
        this.clearSplitterDragState();
        this.setupFlex(true);
    }

    public void toggleLock()
    {
        this.layoutLocked = !this.layoutLocked;
        this.clearPanelDragState();
        this.clearSplitterDragState();
        this.setupFlex(true);
    }

    public void resetLayout()
    {
        this.setLayoutRoot(this.source.getDefault());
        this.refresh();
    }

    /** Current layout tree (with all required panels ensured), e.g. for serializing into a preset. */
    public EditorLayoutNode getLayoutRoot()
    {
        return this.ensureLayoutPanels(this.layoutRoot());
    }

    public void applyLayoutRoot(EditorLayoutNode root)
    {
        if (root != null)
        {
            this.setLayoutRoot(root);
            this.setupFlex(true);
        }
    }

    public boolean cycleDockStackTab(int offset)
    {
        if (offset == 0)
        {
            return false;
        }

        DockStackInfo stack = this.resolveDockStackForKeyboardCycle();

        if (stack == null || !stack.isStacked() || stack.panelIds.isEmpty())
        {
            return false;
        }

        int currentIndex = stack.panelIds.indexOf(stack.activePanelId);

        if (currentIndex < 0)
        {
            currentIndex = 0;
        }

        int size = stack.panelIds.size();
        int nextIndex = (currentIndex + offset) % size;

        if (nextIndex < 0)
        {
            nextIndex += size;
        }

        this.activateDockStackTab(stack.getAnchorPanelId(), stack.panelIds.get(nextIndex));

        return true;
    }

    private DockStackInfo resolveDockStackForKeyboardCycle()
    {
        UIContext context = this.getContext();

        if (context == null)
        {
            return null;
        }

        for (UIDockStackTabs tabs : this.dockStackTabs)
        {
            if (!tabs.isVisible() || !tabs.area.isInside(context.mouseX, context.mouseY))
            {
                continue;
            }

            DockStackInfo hoveredStack = this.dockStackByPanelId.get(tabs.anchorPanelId);

            if (hoveredStack != null && hoveredStack.isStacked())
            {
                return hoveredStack;
            }
        }

        for (Map.Entry<String, UIDockSlot> entry : this.slotById.entrySet())
        {
            UIDockSlot slot = entry.getValue();

            if (!slot.isVisible() || !slot.area.isInside(context.mouseX, context.mouseY))
            {
                continue;
            }

            DockStackInfo stack = this.dockStackByPanelId.get(entry.getKey());

            if (stack != null && stack.isStacked())
            {
                return stack;
            }
        }

        return null;
    }

    private void activateDockStackTab(String stackPanelId, String panelId)
    {
        if (stackPanelId == null || panelId == null)
        {
            return;
        }

        EditorLayoutNode root = this.layoutRoot();
        EditorLayoutNode next = EditorLayoutNode.copyWithStackActivePanel(root, stackPanelId, panelId);

        if (next != root)
        {
            this.setLayoutRoot(next);
            this.setupFlex(true);
        }
    }

    /* Layout build */

    /** Drop what this dock cannot show, apply the host's placement hints, then backstop the rest. */
    private EditorLayoutNode ensureLayoutPanels(EditorLayoutNode root)
    {
        return this.ensureRegisteredPanels(this.ensureFn.apply(this.pruneUnknownPanels(root)));
    }

    /**
     * Panel ids with no registered panel — a layout from another editor, or one renamed since it was
     * saved — would otherwise keep their share of the space as a hole that nothing can be dropped
     * into and only a reset can clear.
     */
    private EditorLayoutNode pruneUnknownPanels(EditorLayoutNode root)
    {
        HashSet<String> ids = new HashSet<>();

        EditorLayoutNode.collectPanelIds(root, ids);

        EditorLayoutNode out = root;

        for (String id : ids)
        {
            if (!this.slotById.containsKey(id))
            {
                out = EditorLayoutNode.copyWithRemovedPanel(out, id);
            }
        }

        return out;
    }

    private EditorLayoutNode ensureRegisteredPanels(EditorLayoutNode root)
    {
        HashSet<String> ids = new HashSet<>();
        EditorLayoutNode.collectPanelIds(root, ids);

        EditorLayoutNode out = root;
        String anchor = null;

        for (String id : ids)
        {
            anchor = id;
            break;
        }

        for (String id : this.slotById.keySet())
        {
            if (!ids.contains(id))
            {
                if (anchor == null)
                {
                    out = new EditorLayoutNode.PanelNode(id);
                    anchor = id;
                }
                else
                {
                    out = EditorLayoutNode.copyWithInsertSplitAt(out, anchor, id, EditorLayoutNode.EDGE_RIGHT);
                }
            }
        }

        return out;
    }

    public void setupFlex(boolean resize)
    {
        EditorLayoutNode originalRoot = this.layoutRoot();
        EditorLayoutNode root = this.ensureLayoutPanels(originalRoot);

        if (root != originalRoot)
        {
            this.setLayoutRoot(root);
        }

        LayoutPass pass = this.computeLayoutPass(root);
        /* Same splitter count means the existing handle elements still map onto the new tree
         * one-for-one; only their bounds and the splitter each one drives have to be refreshed. */
        boolean reuseHandles = resize && pass.handles.size() == this.splitterHandles.size();

        this.splitterHandleInfos.clear();
        this.splitterHandleInfos.addAll(pass.handles);

        if (!reuseHandles)
        {
            this.clearSplitterDragState();

            for (UIDraggable handle : this.splitterHandles)
            {
                handle.removeFromParent();
            }

            this.splitterHandles.clear();

            for (int i = 0; i < pass.handles.size(); i++)
            {
                UIDraggable handle = this.createSplitterHandle(i);

                this.splitterHandles.add(handle);
                this.addBefore(this.dropHighlight, handle);
            }
        }

        this.applyPanelBoundsFromStacks(pass.slots);

        if (!reuseHandles || !this.updateDockStackTabsBoundsOnly(pass.slots))
        {
            this.rebuildDockStackTabs(pass.slots);
        }

        this.syncSplitterHandleBounds();
        this.updateTabVisibility();

        if (resize)
        {
            this.resize();
        }
    }

    /**
     * Walks the tree once and produces everything the pass needs: a slot per panel/stack, a handle
     * per splitter, and each slot's gaps. Gaps come last because they depend on where the frameless
     * panel landed, which is only known once every slot has a rectangle.
     */
    private LayoutPass computeLayoutPass(EditorLayoutNode root)
    {
        LayoutPass pass = new LayoutPass();

        this.collectLayout(root, 0F, 0F, 1F, 1F, pass);

        float[] frameless = null;

        if (this.framelessPanelId != null)
        {
            for (DockStackInfo slot : pass.slots)
            {
                if (slot.panelIds.contains(this.framelessPanelId))
                {
                    frameless = new float[] {slot.x, slot.y, slot.w, slot.h};

                    break;
                }
            }
        }

        for (DockStackInfo slot : pass.slots)
        {
            slot.gutter = this.panelGutter(slot, frameless);
        }

        return pass;
    }

    private void collectLayout(EditorLayoutNode node, float x, float y, float w, float h, LayoutPass out)
    {
        if (node instanceof EditorLayoutNode.PanelNode)
        {
            String panelId = ((EditorLayoutNode.PanelNode) node).getPanelId();
            List<String> ids = new ArrayList<>();

            ids.add(panelId);
            out.slots.add(new DockStackInfo(ids, panelId, x, y, w, h));

            return;
        }

        if (node instanceof EditorLayoutNode.StackNode)
        {
            EditorLayoutNode.StackNode stack = (EditorLayoutNode.StackNode) node;

            out.slots.add(new DockStackInfo(new ArrayList<>(stack.getPanelIds()), stack.getActivePanelId(), x, y, w, h));

            return;
        }

        if (!(node instanceof EditorLayoutNode.SplitterNode))
        {
            return;
        }

        EditorLayoutNode.SplitterNode splitter = (EditorLayoutNode.SplitterNode) node;
        float half = SPLITTER_HANDLE_THICKNESS_NORM * 0.5F;

        if (splitter.isHorizontal())
        {
            float h1 = h * splitter.getRatio();

            out.handles.add(new SplitterHandleInfo(splitter, x, y + h1 - half, w, SPLITTER_HANDLE_THICKNESS_NORM, x, y, w, h, true));
            this.collectLayout(splitter.getFirst(), x, y, w, h1, out);
            this.collectLayout(splitter.getSecond(), x, y + h1, w, h - h1, out);
        }
        else
        {
            float w1 = w * splitter.getRatio();

            out.handles.add(new SplitterHandleInfo(splitter, x + w1 - half, y, SPLITTER_HANDLE_THICKNESS_NORM, h, x, y, w, h, false));
            this.collectLayout(splitter.getFirst(), x, y, w1, h, out);
            this.collectLayout(splitter.getSecond(), x + w1, y, w - w1, h, out);
        }
    }

    private void updateTabVisibility()
    {
        boolean show = this.gate.get();

        for (Map.Entry<String, UIDockSlot> entry : this.slotById.entrySet())
        {
            String panelId = entry.getKey();
            UIDockSlot slot = entry.getValue();
            boolean active = this.isPanelActive(panelId);

            slot.setVisible(show && active);
            slot.dragHandle.setVisible(show && !this.layoutLocked && active);
        }

        for (UIDockStackTabs tabs : this.dockStackTabs)
        {
            tabs.setVisible(show);
        }

        this.onChanged.run();
    }

    /* Splitter handles */

    /**
     * The handle is a fixed-width strip centred on the seam, expressed as the seam's fraction plus
     * a pixel offset. Keeping the pixels in the flex rather than converting them against the current
     * area is what lets a window resize be handled by the flex pass alone.
     */
    private void applySplitterHandleBounds(UIDraggable handle, SplitterHandleInfo info)
    {
        int half = SPLITTER_HANDLE_PX / 2;

        /* Handles are reused across layout changes, so the drag axis has to follow the splitter the
         * handle currently stands for rather than the one it was created for. */
        handle.referenceAxis(!info.horizontal, info.horizontal);

        if (info.horizontal)
        {
            handle.relative(this).x(info.hx).y(info.hy + info.hh * 0.5F, -half).w(info.hw).h(SPLITTER_HANDLE_PX);
        }
        else
        {
            handle.relative(this).x(info.hx + info.hw * 0.5F, -half).y(info.hy).w(SPLITTER_HANDLE_PX).h(info.hh);
        }
    }

    private void syncSplitterHandleBounds()
    {
        for (int i = 0; i < this.splitterHandles.size() && i < this.splitterHandleInfos.size(); i++)
        {
            this.applySplitterHandleBounds(this.splitterHandles.get(i), this.splitterHandleInfos.get(i));
        }
    }

    private UIDraggable createSplitterHandle(int index)
    {
        UIDraggable handle = new UIDraggable((context) -> this.applySplitterDrag(context.mouseX, context.mouseY))
        {
            @Override
            protected boolean subMouseClicked(UIContext context)
            {
                UIDockLayout.this.beginSplitterDrag(index, context.mouseX, context.mouseY);
                boolean handled = super.subMouseClicked(context);

                if (!handled)
                {
                    UIDockLayout.this.clearSplitterDragState();
                }

                return handled;
            }
        };

        /* Disable the handle entirely (no click, no resize cursor) when panel resizing is turned off. */
        handle.enabled(() -> BBSSettings.editorResizablePanels.get());

        handle.dragEnd(() ->
        {
            this.clearSplitterDragState();
            this.onSplitterDragEnd.run();
        });
        handle.reference(() -> this.getSplitterHandleReferencePosition(index));
        handle.rendering((context) -> this.renderSplitter(context, index));

        return handle;
    }

    private void beginSplitterDrag(int index, int mouseX, int mouseY)
    {
        if (!BBSSettings.editorResizablePanels.get() || index < 0 || index >= this.splitterHandleInfos.size())
        {
            this.clearSplitterDragState();
            return;
        }

        this.draggedSplitterIndices.clear();
        this.draggedSplitterIndices.add(index);
        boolean horizontal = this.splitterHandleInfos.get(index).horizontal;

        for (int i = 0; i < this.splitterHandles.size() && i < this.splitterHandleInfos.size(); i++)
        {
            if (i == index || this.splitterHandleInfos.get(i).horizontal == horizontal)
            {
                continue;
            }

            UIDraggable handle = this.splitterHandles.get(i);

            if (this.isInsideSplitterIntersectionHitbox(handle, mouseX, mouseY))
            {
                this.draggedSplitterIndices.add(i);
            }
        }
    }

    private boolean isInsideSplitterIntersectionHitbox(UIDraggable handle, int mouseX, int mouseY)
    {
        int padding = SPLITTER_LINK_HITBOX_PADDING_PX;

        return mouseX >= handle.area.x - padding
            && mouseX < handle.area.ex() + padding
            && mouseY >= handle.area.y - padding
            && mouseY < handle.area.ey() + padding;
    }

    private void clearSplitterDragState()
    {
        this.draggedSplitterIndices.clear();
    }

    /**
     * All dragged splitters are applied in one rebuild: they are identified by node, and rebuilding
     * the path to one of them would replace the nodes the others are still pointing at.
     */
    private void applySplitterDrag(int mouseX, int mouseY)
    {
        if (this.draggedSplitterIndices.isEmpty())
        {
            return;
        }

        Map<EditorLayoutNode.SplitterNode, Float> ratios = new HashMap<>();

        for (int draggedIndex : this.draggedSplitterIndices)
        {
            if (draggedIndex < 0 || draggedIndex >= this.splitterHandleInfos.size())
            {
                continue;
            }

            SplitterHandleInfo info = this.splitterHandleInfos.get(draggedIndex);

            ratios.put(info.node, this.getSplitterRatioFromMouse(info, mouseX, mouseY));
        }

        EditorLayoutNode root = this.layoutRoot();
        EditorLayoutNode next = EditorLayoutNode.copyWithSplitterRatios(root, ratios);

        if (next != root)
        {
            this.setLayoutRoot(next);
            this.setupFlex(true);
        }
    }

    private float getSplitterRatioFromMouse(SplitterHandleInfo info, int mouseX, int mouseY)
    {
        int ex = this.area.x;
        int ey = this.area.y;
        int ew = Math.max(1, this.area.w);
        int eh = Math.max(1, this.area.h);
        float ratio = info.horizontal
            ? (mouseY - (ey + info.py * eh)) / (info.ph * eh)
            : (mouseX - (ex + info.px * ew)) / (info.pw * ew);

        return MathUtils.clamp(ratio, EditorLayoutNode.MIN_RATIO, EditorLayoutNode.MAX_RATIO);
    }

    private Vector2i getSplitterHandleReferencePosition(int index)
    {
        if (index < 0 || index >= this.splitterHandleInfos.size())
        {
            return new Vector2i(this.area.x, this.area.y);
        }

        SplitterHandleInfo info = this.splitterHandleInfos.get(index);
        float r = info.node.getRatio();
        int ex = this.area.x;
        int ey = this.area.y;
        int ew = Math.max(1, this.area.w);
        int eh = Math.max(1, this.area.h);
        int hx = ex + (int) ((info.px + (info.horizontal ? info.pw * 0.5F : r * info.pw)) * ew);
        int hy = ey + (int) ((info.py + (info.horizontal ? r * info.ph : info.ph * 0.5F)) * eh);

        return new Vector2i(hx, hy);
    }

    private void renderSplitter(UIContext context, int index)
    {
        if (index < 0 || index >= this.splitterHandles.size() || index >= this.splitterHandleInfos.size())
        {
            return;
        }

        UIDraggable splitter = this.splitterHandles.get(index);
        SplitterHandleInfo info = this.splitterHandleInfos.get(index);
        int lineColor = BBSSettings.primaryColor(Colors.A100);

        if ((splitter.isDragging() || splitter.area.isInside(context)) && BBSSettings.editorResizablePanels.get())
        {
            context.requestCursor(this.getSplitterCursor(index, context.mouseX, context.mouseY));
        }

        if (!splitter.isDragging() && !this.draggedSplitterIndices.contains(index))
        {
            return;
        }

        if (info.horizontal)
        {
            int cy = splitter.area.y + splitter.area.h / 2;
            int half = SPLITTER_HANDLE_LINE_PX / 2;
            context.batcher.box(splitter.area.x, cy - half, splitter.area.ex(), cy - half + SPLITTER_HANDLE_LINE_PX, lineColor);
        }
        else
        {
            int cx = splitter.area.x + splitter.area.w / 2;
            int half = SPLITTER_HANDLE_LINE_PX / 2;
            context.batcher.box(cx - half, splitter.area.y, cx - half + SPLITTER_HANDLE_LINE_PX, splitter.area.ey(), lineColor);
        }
    }

    private int getSplitterCursor(int index, int mouseX, int mouseY)
    {
        if (index < 0 || index >= this.splitterHandleInfos.size())
        {
            return GLFW.GLFW_ARROW_CURSOR;
        }

        SplitterHandleInfo info = this.splitterHandleInfos.get(index);

        return this.isInsideSplitterIntersection(index, mouseX, mouseY)
            ? GLFW.GLFW_CROSSHAIR_CURSOR
            : info.horizontal
            ? GLFW.GLFW_VRESIZE_CURSOR
            : GLFW.GLFW_HRESIZE_CURSOR;
    }

    private boolean isInsideSplitterIntersection(int index, int mouseX, int mouseY)
    {
        if (index < 0 || index >= this.splitterHandleInfos.size())
        {
            return false;
        }

        boolean horizontal = this.splitterHandleInfos.get(index).horizontal;

        for (int i = 0; i < this.splitterHandles.size() && i < this.splitterHandleInfos.size(); i++)
        {
            if (i == index || this.splitterHandleInfos.get(i).horizontal == horizontal)
            {
                continue;
            }

            if (this.isInsideSplitterIntersectionHitbox(this.splitterHandles.get(i), mouseX, mouseY))
            {
                return true;
            }
        }

        return false;
    }

    /* Dock stacks */

    /**
     * Per-edge gaps so seams between panels don't double up: a full gap where a side does not get
     * a matching half from the other side (the outer edge or the frameless panel), and a half gap
     * where a regular neighbour meets it. Returns left, top, right, bottom offsets in pixels.
     */
    private int[] panelGutter(DockStackInfo info, float[] frameless)
    {
        int half = PANEL_GAP_PX / 2;
        float x = info.x, y = info.y, w = info.w, h = info.h;

        boolean left = x <= PANEL_EDGE_EPS;
        boolean top = y <= PANEL_EDGE_EPS;
        boolean right = x + w >= 1F - PANEL_EDGE_EPS;
        boolean bottom = y + h >= 1F - PANEL_EDGE_EPS;

        if (frameless != null)
        {
            float vx = frameless[0], vy = frameless[1], vw = frameless[2], vh = frameless[3];
            boolean spanY = y < vy + vh - PANEL_EDGE_EPS && y + h > vy + PANEL_EDGE_EPS;
            boolean spanX = x < vx + vw - PANEL_EDGE_EPS && x + w > vx + PANEL_EDGE_EPS;

            left |= spanY && Math.abs(x - (vx + vw)) <= PANEL_EDGE_EPS;
            right |= spanY && Math.abs((x + w) - vx) <= PANEL_EDGE_EPS;
            top |= spanX && Math.abs(y - (vy + vh)) <= PANEL_EDGE_EPS;
            bottom |= spanX && Math.abs((y + h) - vy) <= PANEL_EDGE_EPS;
        }

        return new int[] {
            left ? PANEL_GAP_PX : half,
            top ? PANEL_GAP_PX : half,
            right ? PANEL_GAP_PX : half,
            bottom ? PANEL_GAP_PX : half
        };
    }

    private void applyPanelBoundsFromStacks(List<DockStackInfo> stackInfos)
    {
        this.dockStackByPanelId.clear();

        int inset = this.layoutLocked ? 0 : dragStripHeightPx();

        for (DockStackInfo info : stackInfos)
        {
            int topOffset = info.isStacked() ? DOCK_STACK_TABS_HEIGHT_PX : 0;

            for (String panelId : info.panelIds)
            {
                UIDockSlot slot = this.slotById.get(panelId);

                if (slot == null)
                {
                    continue;
                }

                int[] g = this.isFrameless(panelId) ? NO_GUTTER : info.gutter;

                slot.relative(this)
                    .x(info.x, g[0])
                    .y(info.y, topOffset + g[1])
                    .w(info.w, -g[0] - g[2])
                    .h(info.h, -topOffset - g[1] - g[3]);
                slot.setContentInset(inset);
                this.dockStackByPanelId.put(panelId, info);
            }
        }
    }

    private void rebuildDockStackTabs(List<DockStackInfo> stackInfos)
    {
        for (UIDockStackTabs tabs : this.dockStackTabs)
        {
            tabs.removeFromParent();
        }

        this.dockStackTabs.clear();

        for (DockStackInfo info : stackInfos)
        {
            if (!info.isStacked())
            {
                continue;
            }

            UIDockStackTabs tabs = new UIDockStackTabs(this);
            int[] g = info.gutter;

            tabs.configure(info);

            tabs.relative(this).x(info.x, g[0]).y(info.y, g[1]).w(info.w, -g[0] - g[2]).h(DOCK_STACK_TABS_HEIGHT_PX);
            this.dockStackTabs.add(tabs);
            this.add(tabs);
        }
    }

    private boolean updateDockStackTabsBoundsOnly(List<DockStackInfo> stackInfos)
    {
        List<DockStackInfo> stackedInfos = new ArrayList<>();

        for (DockStackInfo info : stackInfos)
        {
            if (info.isStacked())
            {
                stackedInfos.add(info);
            }
        }

        if (stackedInfos.size() != this.dockStackTabs.size())
        {
            return false;
        }

        for (int i = 0; i < stackedInfos.size(); i++)
        {
            if (!this.dockStackTabs.get(i).matches(stackedInfos.get(i)))
            {
                return false;
            }
        }

        for (int i = 0; i < stackedInfos.size(); i++)
        {
            UIDockStackTabs tabs = this.dockStackTabs.get(i);
            DockStackInfo info = stackedInfos.get(i);
            int[] g = info.gutter;

            tabs.configure(info);

            tabs.relative(this).x(info.x, g[0]).y(info.y, g[1]).w(info.w, -g[0] - g[2]).h(DOCK_STACK_TABS_HEIGHT_PX);
        }

        return true;
    }

    /* Panel drag-to-dock */

    private void clearPanelDragState()
    {
        this.draggingPanelId = null;
        this.dropTargetPanelId = null;
        this.dropTargetZone = DROP_ZONE_CENTER;
    }

    private void applyPanelDropResult(String dragId, String targetId, int zone)
    {
        EditorLayoutNode root = this.layoutRoot();
        EditorLayoutNode newRoot = zone == DROP_ZONE_CENTER
            ? EditorLayoutNode.copyWithInsertStackAt(root, targetId, dragId)
            : EditorLayoutNode.copyWithInsertSplitAt(root, targetId, dragId, zone);

        if (newRoot != null && newRoot != root)
        {
            this.setLayoutRoot(newRoot);
            this.setupFlex(true);
        }
    }

    private UIDraggable createPanelDragHandle(String panelId)
    {
        UIDraggable handle = new UIDraggable((context) ->
        {
            if (this.draggingPanelId == null)
            {
                this.draggingPanelId = panelId;
            }

            this.dropTargetPanelId = null;
            this.dropTargetZone = DROP_ZONE_CENTER;

            for (UIDockStackTabs tabs : this.dockStackTabs)
            {
                if (tabs.isVisible() && tabs.area.isInside(context.mouseX, context.mouseY))
                {
                    String targetPanelId = tabs.getPanelIdAt(context.mouseX);

                    if (targetPanelId != null)
                    {
                        this.dropTargetPanelId = targetPanelId;
                        this.dropTargetZone = DROP_ZONE_CENTER;

                        return;
                    }

                    break;
                }
            }

            /* Slots never overlap, so the first hit is the only hit. */
            for (Map.Entry<String, UIDockSlot> e : this.slotById.entrySet())
            {
                UIDockSlot slot = e.getValue();

                if (slot.isVisible() && slot.area.isInside(context.mouseX, context.mouseY))
                {
                    this.dropTargetPanelId = e.getKey();
                    this.dropTargetZone = this.computeDropZone(slot.area, context.mouseX, context.mouseY);

                    break;
                }
            }
        });

        handle.dragEnd(() ->
        {
            if (this.draggingPanelId == null || this.dropTargetPanelId == null || this.draggingPanelId.equals(this.dropTargetPanelId))
            {
                this.clearPanelDragState();
                return;
            }

            this.applyPanelDropResult(this.draggingPanelId, this.dropTargetPanelId, this.dropTargetZone);
            this.clearPanelDragState();
        });
        handle.hoverOnly().cursors(GLFW.GLFW_HAND_CURSOR, GLFW.GLFW_HAND_CURSOR).rendering((context) -> this.renderPanelDragHandle(context, handle));

        return handle;
    }

    private void renderPanelDragHandle(UIContext context, UIDraggable handle)
    {
        boolean active = handle.area.isInside(context) || handle.isDragging();
        int color = active ? Colors.WHITE : Colors.setA(Colors.WHITE, 0.6F);
        int cx = handle.area.mx();
        int cy = handle.area.y + handle.area.h / 2 + 4;
        context.batcher.icon(Icons.ALL_DIRECTIONS, color, cx, cy, 0.5F, 0.5F);
    }

    private int computeDropZone(Area area, int mouseX, int mouseY)
    {
        int ax = area.x;
        int ay = area.y;
        int aw = area.w;
        int ah = area.h;
        float nx = aw <= 0 ? 0.5F : (mouseX - ax) / (float) aw;
        float ny = ah <= 0 ? 0.5F : (mouseY - ay) / (float) ah;

        if (nx < DROP_EDGE_MARGIN)
        {
            return EditorLayoutNode.EDGE_LEFT;
        }

        if (nx > 1F - DROP_EDGE_MARGIN)
        {
            return EditorLayoutNode.EDGE_RIGHT;
        }

        if (ny < DROP_EDGE_MARGIN)
        {
            return EditorLayoutNode.EDGE_TOP;
        }

        if (ny > 1F - DROP_EDGE_MARGIN)
        {
            return EditorLayoutNode.EDGE_BOTTOM;
        }

        return DROP_ZONE_CENTER;
    }

    /* Rendering */

    private boolean isFrameless(String panelId)
    {
        return this.framelessPanelId != null && this.framelessPanelId.equals(panelId);
    }

    /** The canvas behind the slots; each slot paints its own recessed surface and border. */
    private void renderCanvas(UIContext context)
    {
        this.area.render(context.batcher, BBSSettings.baseSurface());
    }

    private void renderDropZoneHighlight(UIContext context)
    {
        if (this.layoutLocked || this.draggingPanelId == null || this.dropTargetPanelId == null)
        {
            return;
        }

        UIDockSlot target = this.slotById.get(this.dropTargetPanelId);

        if (target == null)
        {
            return;
        }

        Area a = target.area;
        int border = BBSSettings.primaryColor(Colors.A50);
        int fill = BBSSettings.primaryColor(Colors.A25);

        if (this.dropTargetZone == DROP_ZONE_CENTER)
        {
            this.renderDropZoneRect(context, a, border, fill);
            return;
        }

        float m = DROP_EDGE_MARGIN;
        int strip = 2;

        switch (this.dropTargetZone)
        {
            case EditorLayoutNode.EDGE_LEFT:
                context.batcher.box(a.x, a.y, a.x + (int) (a.w * m), a.ey(), fill);
                context.batcher.box(a.x + (int) (a.w * m) - strip, a.y, a.x + (int) (a.w * m) + strip, a.ey(), border);
                break;
            case EditorLayoutNode.EDGE_RIGHT:
                context.batcher.box(a.ex() - (int) (a.w * m), a.y, a.ex(), a.ey(), fill);
                context.batcher.box(a.ex() - (int) (a.w * m) - strip, a.y, a.ex() - (int) (a.w * m) + strip, a.ey(), border);
                break;
            case EditorLayoutNode.EDGE_TOP:
                context.batcher.box(a.x, a.y, a.ex(), a.y + (int) (a.h * m), fill);
                context.batcher.box(a.x, a.y + (int) (a.h * m) - strip, a.ex(), a.y + (int) (a.h * m) + strip, border);
                break;
            case EditorLayoutNode.EDGE_BOTTOM:
                context.batcher.box(a.x, a.ey() - (int) (a.h * m), a.ex(), a.ey(), fill);
                context.batcher.box(a.x, a.ey() - (int) (a.h * m) - strip, a.ex(), a.ey() - (int) (a.h * m) + strip, border);
                break;
            default:
                this.renderDropZoneRect(context, a, border, fill);
                break;
        }
    }

    private void renderDropZoneRect(UIContext context, Area a, int border, int fill)
    {
        context.batcher.box(a.x, a.y, a.ex(), a.ey(), fill);
        int t = 2;
        context.batcher.box(a.x, a.y, a.ex(), a.y + t, border);
        context.batcher.box(a.x, a.ey() - t, a.ex(), a.ey(), border);
        context.batcher.box(a.x, a.y, a.x + t, a.ey(), border);
        context.batcher.box(a.ex() - t, a.y, a.ex(), a.ey(), border);
    }

    /* Helper types */

    /**
     * The frame a panel sits in. Owns everything the dock draws around a panel &mdash; its surface,
     * its border and its drag handle &mdash; and insets the panel itself so the handle never covers
     * content. That inset is why hosts don't need to know the dock is unlocked.
     */
    private static class UIDockSlot extends UIElement
    {
        private final UIDockLayout layout;
        private final String panelId;
        private final UIElement panel;
        private final UIDraggable dragHandle;

        public UIDockSlot(UIDockLayout layout, String panelId, UIElement panel)
        {
            this.layout = layout;
            this.panelId = panelId;
            this.panel = panel;
            this.dragHandle = layout.createPanelDragHandle(panelId);

            panel.relative(this).x(0F).y(0F).w(1F).h(1F);
            this.dragHandle.relative(this).x(0F).y(DRAG_HANDLE_TOP_OFFSET_PX).w(1F).h(DRAG_HANDLE_HEIGHT_PX);

            this.add(panel, this.dragHandle);
        }

        /** Push the panel down by the drag strip while the layout is unlocked. */
        public void setContentInset(int inset)
        {
            this.panel.y(0F, inset).h(1F, -inset);
        }

        public boolean isFramed()
        {
            return !this.layout.isFrameless(this.panelId);
        }

        @Override
        public void render(UIContext context)
        {
            if (this.isFramed())
            {
                this.area.render(context.batcher, BBSSettings.deepSurface());
            }

            super.render(context);

            /* After the children so the inset shadow shows even over panels that paint opaquely. */
            if (this.isFramed() && BBSSettings.interfaceShadows.get())
            {
                int fade = Colors.setA(Colors.A100, 0F);
                Area a = this.area;

                context.batcher.gradientVBox(a.x, a.y, a.ex(), a.y + 4, Colors.A25, fade);
                context.batcher.gradientVBox(a.x, a.ey() - 4, a.ex(), a.ey(), fade, Colors.A25);
                context.batcher.gradientHBox(a.x, a.y, a.x + 4, a.ey(), Colors.A25, fade);
                context.batcher.gradientHBox(a.ex() - 4, a.y, a.ex(), a.ey(), fade, Colors.A25);
            }
        }
    }

    /** Everything one walk of the tree produces. */
    private static class LayoutPass
    {
        public final List<DockStackInfo> slots = new ArrayList<>();
        public final List<SplitterHandleInfo> handles = new ArrayList<>();
    }

    /** One splitter handle: the node it drives, its normalized rect, and its parent's rect. */
    private static class SplitterHandleInfo
    {
        public final EditorLayoutNode.SplitterNode node;
        public final float hx, hy, hw, hh;
        public final float px, py, pw, ph;
        public final boolean horizontal;

        public SplitterHandleInfo(EditorLayoutNode.SplitterNode node, float hx, float hy, float hw, float hh, float px, float py, float pw, float ph, boolean horizontal)
        {
            this.node = node;
            this.hx = hx;
            this.hy = hy;
            this.hw = hw;
            this.hh = hh;
            this.px = px;
            this.py = py;
            this.pw = pw;
            this.ph = ph;
            this.horizontal = horizontal;
        }
    }

    private static class DockStackInfo
    {
        public final List<String> panelIds;
        public final String activePanelId;
        public final float x;
        public final float y;
        public final float w;
        public final float h;
        /** Left, top, right, bottom gaps in pixels; filled in once the whole pass is known. */
        public int[] gutter = NO_GUTTER;

        public DockStackInfo(List<String> panelIds, String activePanelId, float x, float y, float w, float h)
        {
            this.panelIds = panelIds;
            this.activePanelId = activePanelId;
            this.x = x;
            this.y = y;
            this.w = w;
            this.h = h;
        }

        public boolean isStacked()
        {
            return this.panelIds.size() > 1;
        }

        public String getAnchorPanelId()
        {
            return this.panelIds.isEmpty() ? "" : this.panelIds.get(0);
        }
    }

    private static class UIDockStackTabs extends UIElement
    {
        private final UIDockLayout layout;
        private String anchorPanelId = "";
        private final List<String> panelIds = new ArrayList<>();
        private String activePanelId;

        public UIDockStackTabs(UIDockLayout layout)
        {
            this.layout = layout;
        }

        public void configure(DockStackInfo info)
        {
            this.anchorPanelId = info.getAnchorPanelId();
            this.panelIds.clear();
            this.panelIds.addAll(info.panelIds);
            this.activePanelId = info.activePanelId;
            this.setVisible(info.isStacked());
        }

        public boolean matches(DockStackInfo info)
        {
            return this.anchorPanelId.equals(info.getAnchorPanelId()) && this.panelIds.equals(info.panelIds);
        }

        @Override
        public boolean subMouseClicked(UIContext context)
        {
            if (!this.isVisible() || context.mouseButton != 0 || !this.area.isInside(context) || this.panelIds.isEmpty())
            {
                return super.subMouseClicked(context);
            }

            int index = this.getTabIndex(context.mouseX);

            if (index >= 0 && index < this.panelIds.size())
            {
                this.layout.activateDockStackTab(this.anchorPanelId, this.panelIds.get(index));

                return true;
            }

            return super.subMouseClicked(context);
        }

        @Override
        public void render(UIContext context)
        {
            if (!this.isVisible() || this.panelIds.isEmpty())
            {
                return;
            }

            if (this.area.isInside(context))
            {
                context.requestCursor(GLFW.GLFW_HAND_CURSOR);
            }

            int tabSize = this.getTabSize();
            int y = this.area.y;
            int ey = this.area.ey();

            context.batcher.box(this.area.x, this.area.y, this.area.ex(), this.area.ey(), BBSSettings.chromeSurface());

            for (int i = 0; i < this.panelIds.size(); i++)
            {
                int x = this.area.x + i * tabSize;

                if (x >= this.area.ex())
                {
                    break;
                }

                int ex = Math.min(this.area.ex(), x + tabSize);
                String panelId = this.panelIds.get(i);
                boolean active = panelId.equals(this.activePanelId);
                Icon icon = this.layout.getDockPanelIcon(panelId);

                if (active)
                {
                    Area.SHARED.set(x, y, ex - x, ey - y);
                    UIDashboardPanels.renderHighlight(context.batcher, Area.SHARED, Direction.BOTTOM);
                }

                context.batcher.icon(icon, Colors.WHITE, (x + ex) / 2, (y + ey) / 2, 0.5F, 0.5F);
            }

            super.render(context);
        }

        private int getTabSize()
        {
            return Math.max(1, this.area.h);
        }

        private int getTabIndex(int mouseX)
        {
            int index = (mouseX - this.area.x) / this.getTabSize();

            if (index < 0 || index >= this.panelIds.size())
            {
                return -1;
            }

            return index;
        }

        public String getPanelIdAt(int mouseX)
        {
            if (this.panelIds.isEmpty())
            {
                return this.anchorPanelId;
            }

            int index = this.getTabIndex(mouseX);

            if (index < 0)
            {
                return null;
            }

            return this.panelIds.get(index);
        }
    }
}
