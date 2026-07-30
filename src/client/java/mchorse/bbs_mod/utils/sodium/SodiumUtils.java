package mchorse.bbs_mod.utils.sodium;

import me.jellysquid.mods.sodium.client.SodiumClientMod;
import me.jellysquid.mods.sodium.client.gui.SodiumGameOptions;
import mchorse.bbs_mod.forms.renderers.utils.RecolorVertexSodiumConsumer;
import mchorse.bbs_mod.utils.colors.Color;
import net.minecraft.client.render.VertexConsumer;

public class SodiumUtils
{
    private static boolean savedBlockFaceCulling;
    private static boolean savedFogOcclusion;

    public static VertexConsumer createVertexBuffer(VertexConsumer b, Color color)
    {
        return new RecolorVertexSodiumConsumer(b, color);
    }

    /**
     * Turn off Sodium's point-camera culling heuristics for the frame (the
     * orthographic projection breaks their assumptions): the per-section block
     * face culling judges face visibility from the camera POINT, which drops
     * visible faces near the screen edges under parallel sightlines, and the
     * fog occlusion culls whole sections beyond the fog range. The in-memory
     * options are read back every render call, so a per-frame toggle is
     * enough, and Sodium only persists them from its own settings screen.
     */
    public static void disablePointCameraCulling()
    {
        SodiumGameOptions.PerformanceSettings performance = SodiumClientMod.options().performance;

        savedBlockFaceCulling = performance.useBlockFaceCulling;
        savedFogOcclusion = performance.useFogOcclusion;

        performance.useBlockFaceCulling = false;
        performance.useFogOcclusion = false;
    }

    public static void restorePointCameraCulling()
    {
        SodiumGameOptions.PerformanceSettings performance = SodiumClientMod.options().performance;

        performance.useBlockFaceCulling = savedBlockFaceCulling;
        performance.useFogOcclusion = savedFogOcclusion;
    }
}