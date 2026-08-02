package mchorse.bbs_mod.mixin.client;

import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.client.BBSRendering;
import mchorse.bbs_mod.ui.framework.UIScreen;
import net.minecraft.client.Keyboard;
import net.minecraft.client.gui.hud.debug.DebugHudProfile;
import net.minecraft.client.input.KeyInput;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.Redirect;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(Keyboard.class)
public class KeyboardMixin
{
    @Inject(method = "onKey", at = @At("HEAD"))
    public void onOnKey(long window, int action, KeyInput keyInput, CallbackInfo info)
    {
        BBSRendering.lastAction = action;
    }

    /**
     * Swallow the vanilla debug-overlay toggle while a BBS screen is open.
     *
     * <p>1.21.11 toggles it straight from {@code Keyboard#onKey} without checking whether a screen is up, so
     * pressing F3 inside the film or form editor opened Minecraft's debug HUD over the editor. Redirecting
     * just this call leaves every other key path — including the routing that delivers the key to the BBS
     * screen itself — untouched; cancelling onKey outright would have starved the editor of input.
     */
    @Redirect(
        method = "onKey",
        at = @At(value = "INVOKE", target = "Lnet/minecraft/client/gui/hud/debug/DebugHudProfile;toggleF3Enabled()V"),
        require = 0
    )
    private void bbs$skipDebugToggleInEditor(DebugHudProfile profile)
    {
        if (UIScreen.getCurrentMenu() == null)
        {
            profile.toggleF3Enabled();
        }
    }

    @Inject(method = "onKey", at = @At("TAIL"))
    public void onOnEndKey(long window, int action, KeyInput keyInput, CallbackInfo info)
    {
        BBSModClient.onEndKey(window, keyInput.key(), keyInput.scancode(), action, keyInput.modifiers(), info);
    }
}