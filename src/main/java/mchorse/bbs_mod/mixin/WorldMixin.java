package mchorse.bbs_mod.mixin;

import mchorse.bbs_mod.BBSMod;
import mchorse.bbs_mod.actions.ActionManager;
import net.minecraft.block.BlockState;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.BlockPos;
import net.minecraft.world.World;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

@Mixin(World.class)
public class WorldMixin
{
    @Inject(method = "setBlockState(Lnet/minecraft/util/math/BlockPos;Lnet/minecraft/block/BlockState;II)Z", at = @At("HEAD"))
    public void onSetBlockState(BlockPos pos, BlockState state, int flags, int maxUpdateDepth, CallbackInfoReturnable<Boolean> info)
    {
        if ((Object) this instanceof ServerWorld world)
        {
            ActionManager actions = BBSMod.getActions();

            /* Asked before the state and the block entity are looked up, not after: this runs on
             * every block change on the server, and reading them is two chunk lookups that were
             * being paid for even with nothing being filmed and the feature turned off. */
            if (actions == null || !actions.isTracking())
            {
                return;
            }

            actions.changedBlock(pos, world.getBlockState(pos), world.getBlockEntity(pos));
        }
    }
}
