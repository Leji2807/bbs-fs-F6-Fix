package mchorse.bbs_mod.utils;

import com.mojang.authlib.GameProfile;
import mchorse.bbs_mod.film.replays.Replay;
import mchorse.bbs_mod.film.replays.ReplayKeyframes;
import mchorse.bbs_mod.network.ClientNetwork;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.network.ClientPlayerEntity;
import net.minecraft.entity.data.TrackedData;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.util.math.BlockPos;
import net.minecraft.world.World;

public class PlayerUtils
{
    public static void teleport(double x, double y, double z, float yaw, float pitch)
    {
        teleport(x, y, z, yaw, yaw, pitch);
    }

    public static void teleport(double x, double y, double z, float yaw, float bodyYaw, float pitch)
    {
        ClientPlayerEntity player = MinecraftClient.getInstance().player;

        if (!ClientNetwork.isIsBBSModOnServer())
        {
            String command = "tp " + player.getGameProfile().getName() + " " + x + " " + y + " " + z + " " + yaw + " " + pitch;

            player.networkHandler.sendCommand(command);
        }
        else
        {
            ClientNetwork.sendTeleport(x, y, z, yaw, bodyYaw, pitch);
            player.setYaw(yaw);
            player.setHeadYaw(yaw);
            player.setBodyYaw(bodyYaw);
            player.setPitch(pitch);
        }
    }

    /**
     * Put the player exactly where a replay stands at the given tick &mdash; position,
     * yaw, head and body yaw, pitch. This is what the film editor's teleport key does,
     * and what recording in the world does on its own when
     * {@link mchorse.bbs_mod.BBSSettings#recordingTeleport} is on, so the two can never
     * drift apart.
     *
     * <p>A replay that has no transform keyframes at all is left alone: its channels
     * read {@code 0} everywhere, and teleporting would drop the player at the world
     * origin instead of doing nothing.</p>
     *
     * @return whether the player was actually moved
     */
    public static boolean teleportToReplay(Replay replay, int tick)
    {
        ClientPlayerEntity player = MinecraftClient.getInstance().player;
        ReplayKeyframes keyframes = replay == null ? null : replay.keyframes;

        if (player == null || keyframes == null || (keyframes.x.isEmpty() && keyframes.y.isEmpty() && keyframes.z.isEmpty()))
        {
            return false;
        }

        /* Through the replay's own clock, so a looping replay hands over the
         * spot that's actually on screen at that tick */
        int replayTick = replay.getTick(tick);
        double x = keyframes.x.interpolate(replayTick);
        double y = keyframes.y.interpolate(replayTick);
        double z = keyframes.z.interpolate(replayTick);
        float yaw = keyframes.yaw.interpolate(replayTick).floatValue();
        float headYaw = keyframes.headYaw.interpolate(replayTick).floatValue();
        float bodyYaw = keyframes.bodyYaw.interpolate(replayTick).floatValue();
        float pitch = keyframes.pitch.interpolate(replayTick).floatValue();

        teleport(x, y, z, headYaw, pitch);

        player.setYaw(yaw);
        player.setHeadYaw(headYaw);
        player.setBodyYaw(bodyYaw);
        player.setPitch(pitch);

        return true;
    }

    public static void teleport(double x, double y, double z)
    {
        ClientPlayerEntity player = MinecraftClient.getInstance().player;

        if (!ClientNetwork.isIsBBSModOnServer())
        {
            player.networkHandler.sendCommand("tp " + player.getGameProfile().getName() + " " + x + " " + y + " " + z);
        }
        else
        {
            ClientNetwork.sendTeleport(player, x, y, z);
        }
    }

    public static class ProtectedAccess extends PlayerEntity
    {
        public static TrackedData<Byte> getModelParts()
        {
            return PLAYER_MODEL_PARTS;
        }

        public ProtectedAccess(World world, BlockPos pos, float yaw, GameProfile gameProfile)
        {
            super(world, pos, yaw, gameProfile);
        }

        @Override
        public boolean isSpectator()
        {
            return false;
        }

        @Override
        public boolean isCreative()
        {
            return false;
        }
    }
}