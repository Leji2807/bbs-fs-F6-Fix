package mchorse.bbs_mod.mixin.client;

import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.client.BBSRendering;
import net.minecraft.client.util.Window;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.ModifyVariable;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(Window.class)
public class WindowMixin
{
    @Shadow
    private int width;

    @Shadow
    private int height;

    @Shadow
    private int framebufferWidth;

    @Shadow
    private int framebufferHeight;

    @Shadow
    private int scaledWidth;

    @Shadow
    private int scaledHeight;

    @Shadow
    private int scaleFactor;

    /**
     * While BBS UI is open, its ui_scale setting replaces whatever scale vanilla derived from the
     * guiScale option.
     *
     * <p>TODO(1.21.11 render): on 1.21.1 {@code Window.scaleFactor} and {@code setScaleFactor} were
     * doubles, so the whole downstream chain (scaled size, mouse, GUI projection) carried fractional
     * scales for free — that is what made ui_scale a float. 1.21.11 turned both back into {@code int},
     * so a fractional scale can no longer ride this argument and is rounded to the nearest whole step
     * here. The setting still works, just quantised; restoring true fractional scale on this branch
     * means overriding {@code getScaledWidth}/{@code getScaledHeight} (and the mouse/GUI projection
     * that read them) instead of the scale factor itself.
     */
    @ModifyVariable(method = "setScaleFactor", at = @At("HEAD"), argsOnly = true)
    private int bbs$overrideScaleFactor(int scaleFactor)
    {
        float custom = BBSModClient.getCustomGUIScale();

        if (custom > 0F)
        {
            /* Same lower bound vanilla's calculateScaleFactor() enforces: keep at
             * least ~320x240 GUI units on screen, so UI stays usable on small windows */
            int max = Math.max(1, Math.min(this.framebufferWidth / 320, this.framebufferHeight / 240));

            return Math.min(Math.max(Math.round(custom), 1), max);
        }

        return scaleFactor;
    }

    @Inject(method = "getWidth", at = @At("HEAD"), cancellable = true)
    public void onGetWidth(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue(BBSRendering.getVideoWidth());
        }
    }

    @Inject(method = "getHeight", at = @At("HEAD"), cancellable = true)
    public void onGetHeight(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue(BBSRendering.getVideoHeight());
        }
    }

    @Inject(method = "getFramebufferWidth", at = @At("HEAD"), cancellable = true)
    public void onGetFramebufferWidth(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue((int) (BBSRendering.getVideoWidth() * BBSModClient.getOriginalFramebufferScale()));
        }
    }

    @Inject(method = "getFramebufferHeight", at = @At("HEAD"), cancellable = true)
    public void onGetFramebufferHeight(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue((int) (BBSRendering.getVideoHeight() * BBSModClient.getOriginalFramebufferScale()));
        }
    }

    @Inject(method = "getScaledWidth", at = @At("HEAD"), cancellable = true)
    public void onGetScaledWidth(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue((int) (BBSRendering.getVideoWidth() / (double) this.scaleFactor * BBSModClient.getOriginalFramebufferScale()));
        }
    }

    @Inject(method = "getScaledHeight", at = @At("HEAD"), cancellable = true)
    public void onGetScaledHeight(CallbackInfoReturnable<Integer> info)
    {
        if (BBSRendering.canReplaceFramebuffer())
        {
            info.setReturnValue((int) (BBSRendering.getVideoHeight() / (double) this.scaleFactor * BBSModClient.getOriginalFramebufferScale()));
        }
    }
}