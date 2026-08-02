package mchorse.bbs_mod.client.renderer.item;

import mchorse.bbs_mod.BBSMod;
import mchorse.bbs_mod.forms.entities.IEntity;
import mchorse.bbs_mod.forms.entities.StubEntity;
import mchorse.bbs_mod.items.GunProperties;
import net.minecraft.client.MinecraftClient;
import net.minecraft.item.ItemStack;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Holder for the gun's per-stack render state (properties + form entity + expiration).
 * Rendering itself lives in {@link GunSpecialRenderer} (1.21.5+ item-model system); this
 * class keeps the data lifecycle (update ticking, stack lookup) that the renderer and the
 * model-block editor share.
 */
public class GunItemRenderer
{
    private Map<ItemStack, Item> map = new HashMap<>();

    public void update()
    {
        Iterator<Item> it = this.map.values().iterator();

        while (it.hasNext())
        {
            Item item = it.next();

            if (item.expiration <= 0)
            {
                it.remove();
            }

            item.expiration -= 1;
            item.properties.update(item.formEntity);
            item.formEntity.update();
        }
    }

    public Item get(ItemStack stack)
    {
        if (stack == null || stack.getItem() != BBSMod.GUN_ITEM)
        {
            return null;
        }

        if (this.map.containsKey(stack))
        {
            return this.map.get(stack);
        }

        Item item = new Item(GunProperties.get(stack));

        this.map.put(stack, item);

        return item;
    }

    public static class Item
    {
        public GunProperties properties;
        public IEntity formEntity;
        public int expiration = 20;

        public Item(GunProperties properties)
        {
            this.properties = properties;
            this.formEntity = new StubEntity(MinecraftClient.getInstance().world);
        }
    }
}