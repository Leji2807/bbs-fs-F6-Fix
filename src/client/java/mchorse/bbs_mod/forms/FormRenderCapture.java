package mchorse.bbs_mod.forms;

import com.mojang.blaze3d.vertex.VertexFormat;
import com.mojang.blaze3d.vertex.VertexFormatElement;
import mchorse.bbs_mod.forms.entities.IEntity;
import mchorse.bbs_mod.forms.forms.Form;
import mchorse.bbs_mod.forms.renderers.FormRenderType;
import mchorse.bbs_mod.forms.renderers.FormRenderingContext;
import mchorse.bbs_mod.utils.MatrixStackUtils;
import mchorse.bbs_mod.utils.pose.Transform;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.BuiltBuffer;
import net.minecraft.client.render.RenderLayer;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.render.command.OrderedRenderCommandQueue;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.item.ItemDisplayContext;
import org.joml.Vector3f;
import org.joml.Vector3fc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Deferred rendering bridge for BBS forms inside the 1.21.5+ item-model system.
 *
 * <p>BBS's whole form pipeline is immediate: every emitter (cubic models, BOBJ VAOs, the
 * buffered {@link CustomVertexConsumerProvider}, labels, billboards) ends at
 * {@code RenderLayer#draw(BuiltBuffer)}. A {@code SpecialModelRenderer}, on the other hand,
 * is called while the item render state is <em>recorded</em> — no GL pass is open, and drawing
 * immediately lands in the wrong phase (or crashes on a nested pass). The only legal way to
 * draw is to submit commands to the given {@link OrderedRenderCommandQueue}.
 *
 * <p>This class bridges the two: while a capture session is open, the mixin hook in
 * {@code RenderLayerMixin#onDraw} stores the layer and a copy of the bytes instead of drawing;
 * afterwards each captured buffer is submitted as a {@code submitCustom} command and re-emitted
 * vertex by vertex into the queue's consumer at execution time. Vertices were already fully
 * transformed (item display transform + the form's own bone transforms) at capture time, so the
 * raw re-emit needs no matrix of its own.
 *
 * <p>Render thread only. Sessions nest by merging: a form that itself contains a special-model
 * item (gun in a form's hand) re-enters {@code submitForm}, and the inner session folds into the
 * outer one — vertices are fully transformed at capture time, so the outer submit carries them
 * correctly. The depth counter also keeps an unbalanced {@code end()} from leaving the hook armed
 * process-wide (an armed leak silently swallows EVERY RenderLayer draw).
 */
public class FormRenderCapture
{
    private static Map<RenderLayer, List<Captured>> active;
    private static int depth;

    public record Captured(BuiltBuffer.DrawParameters params, ByteBuffer data)
    {}

    public static boolean isActive()
    {
        return active != null;
    }

    public static void begin()
    {
        if (depth++ == 0)
        {
            active = new LinkedHashMap<>();
        }
    }

    /** Returns the captured layers when this call closes the outermost session, null otherwise. */
    public static Map<RenderLayer, List<Captured>> end()
    {
        if (depth == 0)
        {
            return null;
        }

        if (--depth == 0)
        {
            Map<RenderLayer, List<Captured>> result = active;

            active = null;

            return result;
        }

        return null;
    }

    /**
     * Called from {@code RenderLayerMixin#onDraw} while a session is open. Consumes the buffer:
     * vanilla's {@code RenderLayer#draw} closes the {@link BuiltBuffer} it is given, so when the
     * draw is cancelled the capture must close it instead — otherwise the allocator slice leaks
     * ("Clearing BufferBuilder with unused batches").
     */
    public static void capture(RenderLayer layer, BuiltBuffer buffer)
    {
        if (active == null)
        {
            return;
        }

        BuiltBuffer.DrawParameters params = buffer.getDrawParameters();
        /* ByteBuffer.duplicate() does NOT inherit byte order — the duplicate is always BIG_ENDIAN,
         * while BufferBuilder wrote the vertex data in native (little-endian) order. Reading floats
         * through a big-endian view turns 1.0f into 4.6e-41: every position collapses to ~0 and the
         * whole capture rasterises to nothing. Restore native order explicitly on every view. */
        ByteBuffer source = buffer.getBuffer().duplicate().order(ByteOrder.nativeOrder());

        source.limit(Math.min(source.limit(), params.vertexCount() * params.format().getVertexSize()));

        ByteBuffer copy = ByteBuffer.allocate(source.remaining()).order(ByteOrder.nativeOrder());

        copy.put(source);
        copy.flip();

        active.computeIfAbsent(layer, (key) -> new ArrayList<>()).add(new Captured(params, copy));

        buffer.close();
    }

    /**
     * Render the form through the immediate pipeline under a capture session and submit every
     * captured layer to the queue. Applies the same transform the 1.21.1 dynamic item renderer
     * used (item origin at the block corner + the BBS transform of the display context).
     */
    public static void submitForm(Form form, Transform transform, IEntity formEntity, ItemDisplayContext displayContext, MatrixStack matrices, OrderedRenderCommandQueue queue, int light, int overlay)
    {
        if (form == null)
        {
            return;
        }

        matrices.push();
        matrices.translate(0.5F, 0F, 0.5F);
        MatrixStackUtils.applyTransform(matrices, transform);

        begin();

        Map<RenderLayer, List<Captured>> captured;

        try
        {
            FormUtilsClient.render(form, new FormRenderingContext()
                .set(FormRenderType.fromModelMode(displayContext), formEntity, matrices, light, overlay, MinecraftClient.getInstance().getRenderTickCounter().getTickProgress(false))
                .camera(MinecraftClient.getInstance().gameRenderer.getCamera()));
        }
        finally
        {
            matrices.pop();

            /* end() must run even when the form render throws an Error past FormUtilsClient's
             * catch — a session left armed swallows every RenderLayer draw in the process. */
            captured = end();
        }

        if (captured == null)
        {
            /* Nested session: the enclosing submitForm owns the capture and will submit it. */
            return;
        }

        for (Map.Entry<RenderLayer, List<Captured>> entry : captured.entrySet())
        {
            for (Captured single : entry.getValue())
            {
                queue.submitCustom(matrices, entry.getKey(), (matricesEntry, consumer) -> emit(single, consumer));
            }
        }
    }

    /**
     * Re-emit a captured buffer into the queue's consumer. Decodes the vertex format by element
     * usage (POSITION / COLOR / UV0-2 / NORMAL), so any layout the form pipeline produces rides
     * through; unknown elements are skipped.
     */
    public static void emit(Captured captured, VertexConsumer consumer)
    {
        BuiltBuffer.DrawParameters params = captured.params();
        VertexFormat format = params.format();
        int stride = format.getVertexSize();
        /* duplicate() resets byte order to BIG_ENDIAN — see the matching note in capture(). */
        ByteBuffer data = captured.data().duplicate().order(ByteOrder.nativeOrder());

        for (int v = 0; v < params.vertexCount(); v++)
        {
            int base = v * stride;

            for (VertexFormatElement element : format.getElements())
            {
                int offset = base + format.getOffset(element);

                switch (element.usage())
                {
                    case POSITION -> consumer.vertex(data.getFloat(offset), data.getFloat(offset + 4), data.getFloat(offset + 8));
                    case COLOR -> consumer.color(data.get(offset) & 0xFF, data.get(offset + 1) & 0xFF, data.get(offset + 2) & 0xFF, data.get(offset + 3) & 0xFF);
                    case UV ->
                    {
                        if (element.index() == 0)
                        {
                            consumer.texture(data.getFloat(offset), data.getFloat(offset + 4));
                        }
                        else if (element.index() == 1)
                        {
                            consumer.overlay(Short.toUnsignedInt(data.getShort(offset)), Short.toUnsignedInt(data.getShort(offset + 2)));
                        }
                        else if (element.index() == 2)
                        {
                            consumer.light(Short.toUnsignedInt(data.getShort(offset)), Short.toUnsignedInt(data.getShort(offset + 2)));
                        }
                    }
                    case NORMAL -> consumer.normal(data.get(offset) / 127F, data.get(offset + 1) / 127F, data.get(offset + 2) / 127F);
                    default ->
                    {}
                }
            }
        }
    }

    /** Rough item-space bounds for item-frame culling / oversized-GUI detection. */
    public static void collectItemBounds(Consumer<Vector3fc> consumer)
    {
        for (int x = 0; x <= 1; x++)
        {
            for (int y = 0; y <= 1; y++)
            {
                for (int z = 0; z <= 1; z++)
                {
                    consumer.accept(new Vector3f(x, y, z));
                }
            }
        }
    }
}
