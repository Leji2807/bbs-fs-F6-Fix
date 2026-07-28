package mchorse.bbs_mod.cubic.ik;

import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.data.types.MapType;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

final class ModelIKCache
{
    private ModelIKCache()
    {
    }

    public record CompiledChain(String tip, String target, boolean pole, String poleTarget, float poleAngle, float softness, float weight, boolean tipRotation, boolean stretch, boolean classic, List<String> chainRootToEffector)
    {
    }

    public record Compiled(List<CompiledChain> chains, Map<String, ModelIKConfig.JointDoF> bones)
    {
    }

    private static final WeakHashMap<MapType, EmbeddedCompiled> EMBEDDED = new WeakHashMap<>();

    private record EmbeddedCompiled(IModel model, List<CompiledChain> chains, Map<String, ModelIKConfig.JointDoF> bones)
    {
    }

    public static void clear()
    {
        EMBEDDED.clear();
    }

    public static Compiled getFromData(IModel model, MapType data)
    {
        if (model == null || data == null)
        {
            return null;
        }

        EmbeddedCompiled cached = EMBEDDED.get(data);

        if (cached != null && cached.model == model)
        {
            return new Compiled(cached.chains, cached.bones);
        }

        ModelIKConfig config = ModelIKIO.fromData(data);
        List<CompiledChain> compiled = compile(model, config);
        Map<String, ModelIKConfig.JointDoF> bones = config == null || config.bones().isEmpty()
            ? Collections.emptyMap() : Map.copyOf(config.bones());

        EmbeddedCompiled next = new EmbeddedCompiled(model, compiled, bones);
        EMBEDDED.put(data, next);

        return new Compiled(compiled, bones);
    }

    private static List<CompiledChain> compile(IModel model, ModelIKConfig config)
    {
        if (config == null || config.chains() == null || config.chains().isEmpty())
        {
            return Collections.emptyList();
        }

        List<CompiledChain> out = new ArrayList<>(config.chains().size());

        for (ModelIKConfig.Chain chain : config.chains())
        {
            if (chain == null)
            {
                continue;
            }

            if (!chain.enabled())
            {
                continue;
            }

            if (!model.getAllGroupKeys().contains(chain.tip()) || !model.getAllGroupKeys().contains(chain.target()))
            {
                continue;
            }

            List<String> chainIds = buildChainIds(model, chain.tip(), chain.chainLength());

            if (chainIds.size() < 2)
            {
                continue;
            }

            /* Cycle validation, loud: a target that is one of the chain's OWN
             * bones is an absurd rig — the solve's variables move the very point
             * it chases — so such a chain does not compile at all (the panel
             * marks it). A target merely HANGING somewhere under a chain bone is
             * legal and deterministic: frames are collected from the FK pose
             * (orient resets every frame), so the goal is the target's FK spot,
             * never last frame's solve — there is no feedback loop to forbid. */
            if (chainIds.contains(chain.target()))
            {
                continue;
            }

            /* A pole target that does not resolve to a real bone — or that is a
             * chain bone itself (the same absurdity, steering the bend from a
             * point the bend moves) — falls back to the empty pole target: the
             * rest-side virtual pole. */
            String poleTarget = chain.poleTarget();

            if (poleTarget != null && !poleTarget.isEmpty()
                && (!model.getAllGroupKeys().contains(poleTarget) || chainIds.contains(poleTarget)))
            {
                poleTarget = "";
            }

            out.add(new CompiledChain(chain.tip(), chain.target(), chain.pole(), poleTarget, chain.poleAngle(), chain.softness(), chain.weight(), chain.tipRotation(), chain.stretch(), chain.classic(), chainIds));
        }

        return out;
    }

    /** The chain ids the given tip/length setting spans — for the panel's cycle check. */
    public static List<String> chainIdsFor(IModel model, String tip, int chainLength)
    {
        return buildChainIds(model, tip, chainLength);
    }

    /**
     * Walks up the hierarchy from {@code tip}, collecting up to {@code chainLength}
     * bones ({@code 0} = all the way to the root), and returns them ordered
     * root-to-tip.
     */
    private static List<String> buildChainIds(IModel model, String tip, int chainLength)
    {
        List<String> list = new ArrayList<>();
        String group = tip;

        while (group != null && !group.isEmpty())
        {
            list.add(group);

            if (chainLength > 0 && list.size() >= chainLength)
            {
                break;
            }

            String parent = model.getParentGroupKey(group);

            if (parent == null || parent.equals(group))
            {
                break;
            }

            group = parent;
        }

        Collections.reverse(list);

        return list;
    }
}
