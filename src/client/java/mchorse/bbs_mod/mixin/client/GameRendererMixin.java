package mchorse.bbs_mod.mixin.client;

import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.systems.VertexSorter;
import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.BBSSettings;
import mchorse.bbs_mod.camera.controller.CameraController;
import mchorse.bbs_mod.camera.controller.ICameraController;
import mchorse.bbs_mod.camera.controller.PlayCameraController;
import mchorse.bbs_mod.client.BBSRendering;
import mchorse.bbs_mod.items.GunZoom;
import mchorse.bbs_mod.utils.colors.Color;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.GameRenderer;
import net.minecraft.client.render.RenderTickCounter;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.RotationAxis;
import org.joml.Matrix4f;
import org.joml.Vector4f;
import org.objectweb.asm.Opcodes;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.ModifyArg;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(GameRenderer.class)
public class GameRendererMixin
{
    /**
     * This injection cancels bobbing when camera controller takes over
     */
    @Inject(method = "bobView", at = @At("HEAD"), cancellable = true)
    public void onBob(CallbackInfo ci)
    {
        if (BBSModClient.getCameraController().getCurrent() != null)
        {
            ci.cancel();
        }
    }

    /**
     * This injection replaces the camera FOV when camera controller takes over
     */
    @Inject(method = "getFov", at = @At("RETURN"), cancellable = true)
    public void onGetFov(CallbackInfoReturnable<Float> info)
    {
        GunZoom gunZoom = BBSModClient.getGunZoom();

        if (gunZoom != null)
        {
            info.setReturnValue(gunZoom.getFOV(info.getReturnValue()));

            return;
        }

        CameraController controller = BBSModClient.getCameraController();

        if (controller.getCurrent() != null && !BBSRendering.isIrisShadowPass())
        {
            info.setReturnValue((float) controller.getFOV());
        }
    }

    /**
     * This injection replaces the camera roll when camera controller takes over
     */
    @Inject(method = "tiltViewWhenHurt", at = @At("HEAD"), cancellable = true)
    public void onTiltViewWhenHurt(MatrixStack matrices, float tickDelta, CallbackInfo info)
    {
        CameraController controller = BBSModClient.getCameraController();

        if (controller.getCurrent() != null && !BBSRendering.isIrisShadowPass())
        {
            matrices.multiply(RotationAxis.POSITIVE_Z.rotationDegrees(controller.getRoll()));

            info.cancel();
        }
    }

    @Inject(method = "renderHand", at = @At("HEAD"), cancellable = true)
    public void onRenderHand(CallbackInfo info)
    {
        ICameraController current = BBSModClient.getCameraController().getCurrent();

        if (current instanceof PlayCameraController)
        {
            info.cancel();
        }
    }

    @Inject(at = @At("HEAD"), method = "renderWorld")
    private void onWorldRenderBegin(CallbackInfo callbackInfo)
    {
        BBSRendering.onWorldRenderBegin();
    }

    /**
     * These injections substitute an orthographic projection when the film
     * editor's orbit camera asks for one (see BBSRendering#getOrthoProjection).
     * The frustum culling matrix gets the same treatment inside
     * WorldRendererMixin#onSetupFrustumProjection (with a loose lower bound on
     * the frame size, so culling stays conservative when zoomed all the way in).
     */
    /**
     * Record the matrix the world is actually viewed through.
     *
     * <p>{@code BBSRendering.camera} is read by the film editor to place its picking pass and to recover
     * the gizmo's world position, but its only writer — {@code WorldRendererMixin#setupFrustum} — is not
     * registered in {@code bbs.client.mixins.json} on this branch, so it stayed IDENTITY: the picking
     * geometry was drawn with no view at all (nothing was pickable) and the gizmo's drag maths worked off
     * a view-less matrix.
     *
     * <p>Argument 4 of this call is {@code new Matrix4f().rotation(camera.getRotation().conjugate(..))},
     * i.e. the world view — verified against {@code GameRenderer.renderWorld}'s bytecode. Recorded, not
     * modified; the value is returned untouched.
     */
    @ModifyArg(
        method = "renderWorld",
        at = @At(value = "INVOKE", target = "Lnet/minecraft/client/render/WorldRenderer;render(Lnet/minecraft/client/util/memory/ObjectAllocator;Lnet/minecraft/client/render/RenderTickCounter;ZLnet/minecraft/client/render/Camera;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lcom/mojang/blaze3d/buffers/GpuBufferSlice;Lorg/joml/Vector4f;Z)V"),
        index = 4
    )
    private Matrix4f onRenderView(Matrix4f view)
    {
        BBSRendering.camera.set(view);

        return view;
    }

    /* The render call kept its projection in the same argument slot (index 6); only its descriptor
     * changed with the 1.21.5+ render-state rewrite (ObjectAllocator + fog UBO slice + sky colour). */
    @ModifyArg(
        method = "renderWorld",
        at = @At(
            value = "INVOKE",
            target = "Lnet/minecraft/client/render/WorldRenderer;render(Lnet/minecraft/client/util/memory/ObjectAllocator;Lnet/minecraft/client/render/RenderTickCounter;ZLnet/minecraft/client/render/Camera;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lcom/mojang/blaze3d/buffers/GpuBufferSlice;Lorg/joml/Vector4f;Z)V"
        ),
        index = 6
    )
    private Matrix4f onRenderProjection(Matrix4f projection)
    {
        Matrix4f ortho = BBSRendering.getOrthoProjection((GameRenderer) (Object) this, projection, 0F);

        /* Cache what the world is actually drawn with, so the GUI-phase picking passes can bind the same
         * projection instead of the interface ortho that happens to be current there. */
        BBSRendering.setWorldProjection(ortho);

        /* TODO(1.21.11 render): 1.21.1 also pushed the ortho matrix straight into RenderSystem, which now
         * takes a GpuBufferSlice (the projection rides a UBO). The ModifyArg return value below is the one
         * the renderer actually uses, so the push was belt-and-braces; verify nothing downstream read the
         * RenderSystem copy. */
        return ortho;
    }

    /**
     * Chroma sky: feed the world's recorded "clear" pass the chroma colour instead of the fog
     * colour. This {@code Vector4f} argument is consumed by exactly one thing inside
     * {@code WorldRenderer.render} — the lambda of the "clear" pass (verified against the
     * 1.21.11 bytecode) — so substituting it recolours the background and nothing else. The
     * sky pass that would paint over it is cancelled in {@code WorldRendererMixin#onRenderSky},
     * and the fog UBO (a separate argument) is left untouched.
     */
    @ModifyArg(
        method = "renderWorld",
        at = @At(
            value = "INVOKE",
            target = "Lnet/minecraft/client/render/WorldRenderer;render(Lnet/minecraft/client/util/memory/ObjectAllocator;Lnet/minecraft/client/render/RenderTickCounter;ZLnet/minecraft/client/render/Camera;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lorg/joml/Matrix4f;Lcom/mojang/blaze3d/buffers/GpuBufferSlice;Lorg/joml/Vector4f;Z)V"
        ),
        index = 8
    )
    private Vector4f onRenderSkyColor(Vector4f skyColor)
    {
        if (BBSSettings.chromaSkyEnabled.get())
        {
            Integer fromCurve = BBSRendering.getChromaSkyColorArgb();
            int argb = fromCurve != null ? fromCurve : BBSSettings.chromaSkyColor.get();
            Color color = Color.rgba(argb);

            return new Vector4f(color.r, color.g, color.b, 1F);
        }

        return skyColor;
    }

    @Inject(at = @At("RETURN"), method = "renderWorld")
    private void onWorldRenderEnd(CallbackInfo callbackInfo)
    {
        BBSRendering.onWorldRenderEnd();
    }

    @Inject(method = "render", at = @At(value = "INVOKE", target = "Lnet/minecraft/client/gui/hud/InGameHud;render(Lnet/minecraft/client/gui/DrawContext;Lnet/minecraft/client/render/RenderTickCounter;)V", ordinal = 0), require = 0)
    private void onBeforeHudRendering(RenderTickCounter tickCounter, boolean tick, CallbackInfo info)
    {
        ICameraController current = BBSModClient.getCameraController().getCurrent();

        if (MinecraftClient.getInstance().options.hudHidden && current == null)
        {
            BBSRendering.onRenderBeforeScreen();
        }
    }
}