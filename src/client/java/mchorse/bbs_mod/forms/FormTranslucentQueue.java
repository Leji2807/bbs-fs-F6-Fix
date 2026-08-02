package mchorse.bbs_mod.forms;

import org.joml.Vector3f;

/**
 * Two-pass translucency for forms &mdash; <strong>disabled on the 1.21.11 branch</strong>.
 *
 * <p>On 1.21.1 this was the deferred translucent pass: a form whose texture carries
 * semi-transparent texels drew twice — the opaque texels immediately (writing depth) and the
 * semi-transparent ones from a retained buffer replayed at the end of the frame, sorted
 * far-to-near, so translucent forms blended against each other instead of occluding whatever was
 * behind them. A uniform colour fade deferred the whole draw with depth kept on, so the faded
 * model still self-occluded.
 *
 * <p>TODO(1.21.11 render): the mechanism rests on three APIs the 1.21.5+ GPU-pipeline rewrite
 * removed:
 * <ul>
 *   <li>{@code net.minecraft.client.gl.VertexBuffer} — retained the geometry for the replay;</li>
 *   <li>{@code BufferRenderer.drawWithGlobalProgram} / {@code RenderSystem.setShader} — issued the
 *       replay draw with a chosen program;</li>
 *   <li>mutable {@code GlUniform}s — carried the {@code PassMode} split into the shader (a custom
 *       uniform must now ride a std140 UBO declared on the {@code RenderPipeline}).</li>
 * </ul>
 * Re-porting it means a {@code PassMode} UBO entry plus one pipeline variant per pass, and
 * rebuilding the command queue on {@code RenderLayer}/{@code BuiltBuffer}. Until then forms draw
 * single-pass — which is what this branch's renderers already did before the 1.21.1 merge, so it
 * is a feature not yet carried over rather than a regression against the port.
 *
 * <p>What is left below is the inert facade that keeps the merged call sites honest: the queue
 * reports itself inactive, nothing is ever enqueued, and the sort origin is still tracked because
 * {@link CustomVertexConsumerProvider} and the form renderers publish it unconditionally. The full
 * original implementation is preserved in git history (this file on the {@code 1.21.1} branch).
 */
public class FormTranslucentQueue
{
    public static final int PASS_SINGLE = 0;
    public static final int PASS_OPAQUE = 1;
    public static final int PASS_TRANSLUCENT = 2;

    /**
     * Camera-space origin of the form currently being drawn through the buffered vertex consumer
     * path (blocks, items). Still tracked because the renderers publish it unconditionally, and a
     * re-port will need it back as the deferred sort key.
     */
    private static Vector3f sortOrigin;

    public static void setSortOrigin(Vector3f origin)
    {
        sortOrigin = origin;
    }

    public static Vector3f getSortOrigin()
    {
        return sortOrigin;
    }

    /** Always {@code false}: with the deferred pass off, no group is ever recorded. */
    public static boolean isGroupOpen()
    {
        return false;
    }

    public static void beginGroup(Vector3f cameraSpaceOrigin, boolean cull)
    {}

    public static void endGroup()
    {}

    /** Always {@code false} on 1.21.11 — every caller falls through to its immediate draw. */
    public static boolean isActive()
    {
        return false;
    }

    public static void begin()
    {}

    public static boolean suspend()
    {
        return false;
    }

    public static void restore(boolean wasActive)
    {}

    public static void flush()
    {
        sortOrigin = null;
    }
}
