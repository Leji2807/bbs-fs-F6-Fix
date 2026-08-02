package mchorse.bbs_mod.graphics.texture;

import com.mojang.logging.LogUtils;
import org.lwjgl.opengl.GL11;
import org.slf4j.Logger;

import java.util.HashSet;
import java.util.Set;

/**
 * TEMPORARY diagnostic for the 1.21.11 port: reports who turns a texture's filter to LINEAR.
 *
 * <p>Every distinct (texture, old filter, new filter, caller) combination is logged ONCE, so a change that
 * happens every frame does not drown the log. Delete this class and its two call sites once the cause is
 * found.
 */
public class TextureFilterLog
{
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final Set<String> SEEN = new HashSet<>();

    /** Set false to silence without removing the wiring. */
    public static boolean enabled = true;

    public static void record(Texture texture, int from, int to)
    {
        if (!enabled || from == to)
        {
            return;
        }

        String name = texture.debugName == null ? "<unnamed>" : texture.debugName;
        String caller = caller();
        String key = name + "|" + texture.id + "|" + from + ">" + to + "|" + caller;

        if (!SEEN.add(key))
        {
            return;
        }

        LOGGER.info("[BBS filter] {} (glId={} {}x{}) {} -> {}\n    via {}",
            name, texture.id, texture.width, texture.height, describe(from), describe(to), caller);
    }

    /** Adoption into a vanilla sampler bakes the filter in; report those too. */
    public static void recordAdopt(String what, int glId, boolean linear)
    {
        if (!enabled)
        {
            return;
        }

        String caller = caller();
        String key = "adopt|" + what + "|" + glId + "|" + linear + "|" + caller;

        if (!SEEN.add(key))
        {
            return;
        }

        LOGGER.info("[BBS filter] adopt {} (glId={}) as {}\n    via {}",
            what, glId, linear ? "LINEAR" : "NEAREST", caller);
    }

    /**
     * Read back what GL ACTUALLY has on the bound texture and shout if it disagrees with what BBS believes.
     * Everything BBS asks for now says NEAREST, so if a texture still samples blurred the disagreement has
     * to be here — either something outside BBS changed the parameters, or a vanilla sampler object is
     * overriding them.
     */
    public static void verifyBound(Texture texture)
    {
        if (!enabled || texture.id < 0)
        {
            return;
        }

        int min = GL11.glGetTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER);
        int mag = GL11.glGetTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER);

        if (min == texture.getFilter() && mag == texture.getFilter())
        {
            return;
        }

        String name = texture.debugName == null ? "<unnamed>" : texture.debugName;
        String key = "mismatch|" + name + "|" + texture.id + "|" + min + "|" + mag;

        if (!SEEN.add(key))
        {
            return;
        }

        LOGGER.warn("[BBS filter] MISMATCH {} (glId={}) BBS thinks {}, GL has min={} mag={}\n    via {}",
            name, texture.id, describe(texture.getFilter()), describe(min), describe(mag), caller());
    }

    private static String describe(int filter)
    {
        if (filter == GL11.GL_LINEAR) return "LINEAR";
        if (filter == GL11.GL_NEAREST) return "NEAREST";

        return "0x" + Integer.toHexString(filter);
    }

    /** The first few frames outside this package — i.e. whoever actually asked for the change. */
    private static String caller()
    {
        StringBuilder builder = new StringBuilder();
        int shown = 0;

        for (StackTraceElement element : new Throwable().getStackTrace())
        {
            String cls = element.getClassName();

            if (cls.startsWith("mchorse.bbs_mod.graphics.texture."))
            {
                continue;
            }

            if (!cls.startsWith("mchorse.bbs_mod") && shown == 0)
            {
                continue;
            }

            builder.append(shown == 0 ? "" : "\n         <- ").append(element);

            if (++shown >= 5)
            {
                break;
            }
        }

        return builder.length() == 0 ? "<no bbs frames>" : builder.toString();
    }
}
