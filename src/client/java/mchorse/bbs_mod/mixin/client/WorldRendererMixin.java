package mchorse.bbs_mod.mixin.client;

import com.mojang.blaze3d.buffers.GpuBufferSlice;
import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.client.BBSRendering;
import mchorse.bbs_mod.forms.FormTranslucentQueue;
import net.minecraft.client.gl.Framebuffer;
import net.minecraft.client.render.Camera;
import net.minecraft.client.render.FrameGraphBuilder;
import net.minecraft.client.render.WorldRenderer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(WorldRenderer.class)
public class WorldRendererMixin
{
    /* 1.21.11 renamed and privatized the outlines framebuffer field. */
    @Shadow
    private Framebuffer entityOutlineFramebuffer;

    /* Deferred form translucency spans the frame: forms enqueue their translucent pass while
     * entities render, and the queue flushes right before the translucent terrain layer so the
     * blending sits under water/glass the way vanilla entities do. The RETURN hook is a safety
     * net for frames where the translucent layer never draws (e.g. a replaced terrain pipeline).
     * Both are no-ops while FormTranslucentQueue is an inert facade on this branch. */
    @Inject(method = "render", at = @At("HEAD"))
    public void onRenderWorldStart(CallbackInfo info)
    {
        FormTranslucentQueue.begin();
    }

    @Inject(method = "render", at = @At("RETURN"))
    public void onRenderWorldEnd(CallbackInfo info)
    {
        FormTranslucentQueue.flush();
    }

    /**
     * Chroma sky: skip the vanilla sky pass so the frame keeps the flat chroma colour that the
     * recorded "clear" pass was fed with (GameRendererMixin substitutes the fog/clear colour
     * argument of {@code WorldRenderer.render} when chroma is enabled).
     *
     * <p>On 1.21.1 this hook cleared the colour buffer by hand at {@code renderSky} HEAD; in the
     * frame-graph world the clear is a recorded pass of its own, so the hook shrinks to cancelling
     * the sky geometry. Terrain hiding moved to {@code SectionRenderStateMixin} (the old
     * {@code renderLayer} chunk hook is gone with the chunk {@code RenderLayer} pipeline), and the
     * fog UBO is intentionally left alone — terrain, when shown, still fades toward the real fog
     * colour rather than the chroma colour.
     */
    @Inject(
        method = "renderSky(Lnet/minecraft/client/render/FrameGraphBuilder;Lnet/minecraft/client/render/Camera;Lcom/mojang/blaze3d/buffers/GpuBufferSlice;)V",
        at = @At("HEAD"),
        cancellable = true
    )
    private void onRenderSky(FrameGraphBuilder frameGraphBuilder, Camera camera, GpuBufferSlice fog, CallbackInfo info)
    {
        if (BBSSettings.chromaSkyEnabled.get())
        {
            info.cancel();
        }
    }

    @Inject(at = @At("RETURN"), method = "loadEntityOutlinePostProcessor")
    private void onLoadEntityOutlineShader(CallbackInfo info)
    {
        BBSRendering.resizeExtraFramebuffers();
    }

    @Inject(at = @At("RETURN"), method = "onResized")
    private void onResized(CallbackInfo info)
    {
        if (this.entityOutlineFramebuffer == null)
        {
            return;
        }

        BBSRendering.resizeExtraFramebuffers();
    }
}
