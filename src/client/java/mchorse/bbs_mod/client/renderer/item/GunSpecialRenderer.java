package mchorse.bbs_mod.client.renderer.item;

import com.mojang.serialization.MapCodec;
import mchorse.bbs_mod.BBSModClient;
import mchorse.bbs_mod.forms.FormRenderCapture;
import mchorse.bbs_mod.forms.forms.Form;
import mchorse.bbs_mod.items.GunProperties;
import mchorse.bbs_mod.ui.framework.UIScreen;
import mchorse.bbs_mod.ui.model_blocks.UIModelBlockEditorMenu;
import mchorse.bbs_mod.utils.pose.Transform;
import net.minecraft.client.render.command.OrderedRenderCommandQueue;
import net.minecraft.client.render.item.model.special.SpecialModelRenderer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.item.ItemDisplayContext;
import net.minecraft.item.ItemStack;
import org.joml.Vector3fc;

import java.util.function.Consumer;

/**
 * Gun item rendering on the 1.21.5+ item-model system: renders the gun's BBS {@link Form}
 * (with the first-person zoom form and the zoom-section editor preview, as the 1.21.1
 * {@code GunItemRenderer} did) by capturing the immediate form pipeline and replaying it
 * into the item command queue — see {@link FormRenderCapture}.
 */
public class GunSpecialRenderer implements SpecialModelRenderer<GunItemRenderer.Item>
{
    @Override
    public GunItemRenderer.Item getData(ItemStack stack)
    {
        return BBSModClient.getGunItemRenderer().get(stack);
    }

    @Override
    public void render(GunItemRenderer.Item item, ItemDisplayContext displayContext, MatrixStack matrices, OrderedRenderCommandQueue queue, int light, int overlay, boolean glint, int outlineColor)
    {
        if (item == null)
        {
            return;
        }

        GunProperties properties = item.properties;
        Form form = properties.getForm(displayContext);
        Transform transform = properties.getTransform(displayContext);
        boolean zoom = displayContext.isFirstPerson() && BBSModClient.getGunZoom() != null && properties.getZoomForm() != null;

        if (zoom)
        {
            form = properties.getZoomForm();
            transform = properties.zoomTransform;
        }

        /* Preview zoom form */
        if (UIScreen.getCurrentMenu() instanceof UIModelBlockEditorMenu editorMenu && editorMenu.currentSection == editorMenu.sectionZoom)
        {
            form = editorMenu.getGunProperties().getZoomForm();
            transform = editorMenu.getGunProperties().zoomTransform;
        }

        if (form != null)
        {
            item.expiration = 20;

            FormRenderCapture.submitForm(form, transform, item.formEntity, displayContext, matrices, queue, light, overlay);
        }
    }

    @Override
    public void collectVertices(Consumer<Vector3fc> consumer)
    {
        FormRenderCapture.collectItemBounds(consumer);
    }

    public static class Unbaked implements SpecialModelRenderer.Unbaked
    {
        public static final MapCodec<GunSpecialRenderer.Unbaked> CODEC = MapCodec.unit(new GunSpecialRenderer.Unbaked());

        @Override
        public SpecialModelRenderer<?> bake(SpecialModelRenderer.BakeContext context)
        {
            return new GunSpecialRenderer();
        }

        @Override
        public MapCodec<? extends SpecialModelRenderer.Unbaked> getCodec()
        {
            return CODEC;
        }
    }
}
