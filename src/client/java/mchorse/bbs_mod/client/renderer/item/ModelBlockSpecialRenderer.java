package mchorse.bbs_mod.client.renderer.item;

import com.mojang.serialization.MapCodec;
import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.blocks.entities.ModelProperties;
import mchorse.bbs_mod.forms.FormRenderCapture;
import mchorse.bbs_mod.forms.forms.Form;
import net.minecraft.client.render.command.OrderedRenderCommandQueue;
import net.minecraft.client.render.item.model.special.SpecialModelRenderer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.item.ItemDisplayContext;
import net.minecraft.item.ItemStack;
import org.joml.Vector3fc;

import java.util.function.Consumer;

/**
 * Model-block item rendering on the 1.21.5+ item-model system: renders the block's BBS
 * {@link Form} (as the 1.21.1 {@code ModelBlockItemRenderer} did) by capturing the immediate
 * form pipeline and replaying it into the item command queue — see {@link FormRenderCapture}.
 */
public class ModelBlockSpecialRenderer implements SpecialModelRenderer<ModelBlockItemRenderer.Item>
{
    @Override
    public ModelBlockItemRenderer.Item getData(ItemStack stack)
    {
        return BBSModClient.getModelBlockItemRenderer().get(stack);
    }

    @Override
    public void render(ModelBlockItemRenderer.Item item, ItemDisplayContext displayContext, MatrixStack matrices, OrderedRenderCommandQueue queue, int light, int overlay, boolean glint, int outlineColor)
    {
        if (item == null)
        {
            return;
        }

        ModelProperties properties = item.entity.getProperties();
        Form form = properties.getForm(displayContext);

        if (form != null)
        {
            item.expiration = 20;

            FormRenderCapture.submitForm(form, properties.getTransform(displayContext), item.formEntity, displayContext, matrices, queue, light, overlay);
        }
    }

    @Override
    public void collectVertices(Consumer<Vector3fc> consumer)
    {
        FormRenderCapture.collectItemBounds(consumer);
    }

    public static class Unbaked implements SpecialModelRenderer.Unbaked
    {
        public static final MapCodec<ModelBlockSpecialRenderer.Unbaked> CODEC = MapCodec.unit(new ModelBlockSpecialRenderer.Unbaked());

        @Override
        public SpecialModelRenderer<?> bake(SpecialModelRenderer.BakeContext context)
        {
            return new ModelBlockSpecialRenderer();
        }

        @Override
        public MapCodec<? extends SpecialModelRenderer.Unbaked> getCodec()
        {
            return CODEC;
        }
    }
}
