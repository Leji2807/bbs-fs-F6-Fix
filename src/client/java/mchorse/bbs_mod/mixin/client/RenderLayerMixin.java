package mchorse.bbs_mod.mixin.client;

import mchorse.bbs_mod.forms.CustomVertexConsumerProvider;
import net.minecraft.client.render.BuiltBuffer;
import net.minecraft.client.render.RenderLayer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(RenderLayer.class)
public class RenderLayerMixin
{
    /* This hook exists so that form renderers can override the GL state vanilla just set up for
     * the layer (custom texture for mob forms, the picker shader/program, blending). That only
     * works if it runs AFTER RenderLayer#startDrawing: the layer's own phases set the shader,
     * the shader texture and the transparency there, so anything applied at HEAD is immediately
     * overwritten by vanilla and never reaches the draw call. */
    @Inject(method = "draw", at = @At(value = "INVOKE", target = "Lnet/minecraft/client/render/RenderLayer;startDrawing()V", ordinal = 0, shift = At.Shift.AFTER))
    public void onDraw(BuiltBuffer buffer, CallbackInfo info)
    {
        CustomVertexConsumerProvider.drawLayer((RenderLayer) (Object) this);
    }
}
