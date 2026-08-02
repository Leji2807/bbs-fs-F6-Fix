package mchorse.bbs_mod.mixin.client;

import mchorse.bbs_mod.forms.CustomVertexConsumerProvider;
import mchorse.bbs_mod.forms.FormRenderCapture;
import net.minecraft.client.render.BuiltBuffer;
import net.minecraft.client.render.RenderLayer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(RenderLayer.class)
public class RenderLayerMixin
{
    /* This hook lets form renderers override what vanilla set up for the layer (custom texture for
     * mob forms, the picker program, blending).
     *
     * TODO(1.21.11 render): on 1.21.1 it had to fire AFTER RenderLayer#startDrawing, because the
     * layer's phases imperatively set the shader, its texture and the transparency there, so
     * anything applied at HEAD was immediately overwritten. 1.21.5+ removed startDrawing entirely —
     * that state is baked into the layer's RenderPipeline and no longer settable per draw — so the
     * injection goes back to HEAD (the port's original point). Verify in game that the mob-form
     * texture override still lands.
     *
     * While a FormRenderCapture session is open (deferred item-model rendering), the draw is
     * captured instead of executed — no GL pass is open at item-record time, so an immediate
     * draw here would land in the wrong phase. */
    @Inject(method = "draw", at = @At("HEAD"), cancellable = true)
    public void onDraw(BuiltBuffer buffer, CallbackInfo info)
    {
        if (FormRenderCapture.isActive())
        {
            FormRenderCapture.capture((RenderLayer) (Object) this, buffer);

            info.cancel();

            return;
        }

        CustomVertexConsumerProvider.drawLayer((RenderLayer) (Object) this);
    }
}
