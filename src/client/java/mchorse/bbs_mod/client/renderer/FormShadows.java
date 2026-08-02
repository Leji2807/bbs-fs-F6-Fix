package mchorse.bbs_mod.client.renderer;

import net.minecraft.block.BlockRenderType;
import net.minecraft.block.BlockState;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.BufferBuilder;
import net.minecraft.client.render.LightmapTextureManager;
import net.minecraft.client.render.OverlayTexture;
import net.minecraft.client.render.RenderLayers;
import net.minecraft.client.render.Tessellator;
import net.minecraft.client.render.VertexFormats;
import net.minecraft.client.render.entity.state.EntityRenderState;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.ColorHelper;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.chunk.WorldChunk;
import org.joml.Matrix4f;

import java.util.ArrayList;
import java.util.List;

/**
 * Entity-style ground shadows for BBS-owned renderables (film replays, model blocks), ported
 * onto the 1.21.5+ render-state pipeline.
 *
 * <p>Vanilla 1.21.1 exposed {@code EntityRenderDispatcher#renderShadow}, which BBS used to call
 * with a fake actor entity. The render-state rewrite removed it: shadows are now recorded as
 * {@link EntityRenderState.ShadowPiece}s (built in {@code EntityRenderer#updateShadow} /
 * {@code addShadowPiece}) and drawn by {@code ShadowPiecesCommandRenderer} through the command
 * queue. This class mirrors both halves, byte-for-byte in math, so BBS callers can either submit
 * the pieces to a queue (model-block path) or draw them immediately (film's AFTER_ENTITIES
 * context, where BBS's whole form pipeline is immediate).
 */
public class FormShadows
{
    /**
     * Build the shadow pieces under ({@code x}, {@code y}, {@code z}) — the vanilla
     * {@code updateShadow}/{@code addShadowPiece} logic: scan the column region beneath the
     * position, keep full-cube lit blocks, fade the alpha with camera distance, depth below
     * the entity and sky brightness.
     */
    public static List<EntityRenderState.ShadowPiece> buildPieces(ClientWorld world, double x, double y, double z, float radius, float opacity, double squaredDistanceToCamera)
    {
        List<EntityRenderState.ShadowPiece> pieces = new ArrayList<>();

        if (!MinecraftClient.getInstance().options.getEntityShadows().getValue())
        {
            return pieces;
        }

        radius = Math.min(radius, 32F);

        float alpha = (float) ((1D - squaredDistanceToCamera / 256D) * opacity);

        if (radius <= 0F || alpha <= 0F)
        {
            return pieces;
        }

        int minX = MathHelper.floor(x - radius);
        int maxX = MathHelper.floor(x + radius);
        int minZ = MathHelper.floor(z - radius);
        int maxZ = MathHelper.floor(z + radius);
        int minY = MathHelper.floor(y - Math.min(alpha / 0.5F - 1F, radius));
        int maxY = MathHelper.floor(y);

        BlockPos.Mutable mutable = new BlockPos.Mutable();

        for (int bx = minX; bx <= maxX; bx++)
        {
            for (int bz = minZ; bz <= maxZ; bz++)
            {
                WorldChunk chunk = world.getChunk(bx >> 4, bz >> 4);

                for (int by = maxY; by >= minY; by--)
                {
                    addPiece(pieces, world, chunk, mutable.set(bx, by, bz), x, y, z, alpha);
                }
            }
        }

        return pieces;
    }

    /** One vanilla {@code addShadowPiece}: block-under checks + per-piece alpha. */
    private static void addPiece(List<EntityRenderState.ShadowPiece> pieces, ClientWorld world, WorldChunk chunk, BlockPos.Mutable pos, double x, double y, double z, float alpha)
    {
        float pieceAlpha = alpha - (float) (y - pos.getY()) * 0.5F;

        BlockPos down = pos.down();
        BlockState state = chunk.getBlockState(down);

        if (state.getRenderType() == BlockRenderType.INVISIBLE)
        {
            return;
        }

        int light = world.getLightLevel(pos);

        if (light <= 3)
        {
            return;
        }

        if (!state.isFullCube(chunk, down))
        {
            return;
        }

        VoxelShape shape = state.getOutlineShape(chunk, down);

        if (shape.isEmpty())
        {
            return;
        }

        float brightness = MathHelper.clamp(pieceAlpha * 0.5F * LightmapTextureManager.getBrightness(world.getDimension(), light), 0F, 1F);

        pieces.add(new EntityRenderState.ShadowPiece((float) (pos.getX() - x), (float) (pos.getY() - y), (float) (pos.getZ() - z), shape, brightness));
    }

    /**
     * Draw the pieces immediately through the entity-shadow layer — the same quads
     * {@code ShadowPiecesCommandRenderer} emits, for contexts with no command queue
     * (the film's immediate AFTER_ENTITIES rendering).
     */
    public static void drawImmediate(MatrixStack matrices, List<EntityRenderState.ShadowPiece> pieces, float radius)
    {
        if (pieces.isEmpty())
        {
            return;
        }

        BufferBuilder builder = Tessellator.getInstance().begin(com.mojang.blaze3d.vertex.VertexFormat.DrawMode.QUADS, VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL);
        Matrix4f matrix = matrices.peek().getPositionMatrix();

        for (EntityRenderState.ShadowPiece piece : pieces)
        {
            Box box = piece.shapeBelow().getBoundingBox();

            float x1 = piece.relativeX() + (float) box.minX;
            float x2 = piece.relativeX() + (float) box.maxX;
            float y1 = piece.relativeY() + (float) box.minY;
            float z1 = piece.relativeZ() + (float) box.minZ;
            float z2 = piece.relativeZ() + (float) box.maxZ;
            float u1 = -x1 / 2F / radius + 0.5F;
            float u2 = -x2 / 2F / radius + 0.5F;
            float v1 = -z1 / 2F / radius + 0.5F;
            float v2 = -z2 / 2F / radius + 0.5F;
            int color = ColorHelper.getWhite(piece.alpha());

            vertex(builder, matrix, color, x1, y1, z1, u1, v1);
            vertex(builder, matrix, color, x1, y1, z2, u1, v2);
            vertex(builder, matrix, color, x2, y1, z2, u2, v2);
            vertex(builder, matrix, color, x2, y1, z1, u2, v1);
        }

        RenderLayers.entityShadow(net.minecraft.util.Identifier.ofVanilla("textures/misc/shadow.png")).draw(builder.end());
    }

    private static void vertex(BufferBuilder builder, Matrix4f matrix, int color, float x, float y, float z, float u, float v)
    {
        builder.vertex(matrix, x, y, z).color(color).texture(u, v).overlay(OverlayTexture.DEFAULT_UV).light(15728880).normal(0F, 1F, 0F);
    }
}
