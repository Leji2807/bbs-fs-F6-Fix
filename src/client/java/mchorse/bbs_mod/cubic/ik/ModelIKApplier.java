package mchorse.bbs_mod.cubic.ik;

import mchorse.bbs_mod.bobj.BOBJBone;
import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.cubic.constraints.ModelConstraintsConfig.BoneConstraint;
import mchorse.bbs_mod.cubic.data.model.Model;
import mchorse.bbs_mod.cubic.data.model.ModelGroup;
import mchorse.bbs_mod.cubic.ik.solver.IKJoint;
import mchorse.bbs_mod.cubic.ik.solver.IKTree;
import mchorse.bbs_mod.cubic.ik.solver.IKTreeSolver;
import mchorse.bbs_mod.cubic.model.bobj.BOBJModel;
import mchorse.bbs_mod.cubic.render.CubicRenderer.PivotFrame;
import mchorse.bbs_mod.cubic.render.ModelPivotFrames;
import mchorse.bbs_mod.cubic.render.ModelRotationBlender;
import mchorse.bbs_mod.utils.joml.Matrices;
import mchorse.bbs_mod.utils.pose.Transform;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

final class ModelIKApplier
{
    /**
     * Per-frame solve dump to {@code run/ik-log.txt} (truncated every frame, so the
     * file always holds the LAST frame's trees) — the IK counterpart of the drag
     * log, and like it the thing to ask for when a solve is disputed: captured
     * angles in, goals, iterations, errors, angles out. Flip to enable; zero cost off.
     */
    private static final boolean LOG_IK = false;

    private static final StringBuilder LOG = new StringBuilder();

    private static final float EPS = 1.0e-6f;

    /* Legacy BOBJ position-level solve (replaced at the BOBJ port milestone). */
    private static final int LEGACY_MAX_ITERATIONS = 12;
    private static final float LEGACY_TOLERANCE = 1.0e-4f;

    private ModelIKApplier()
    {
    }

    /** A chain's per-frame solve inputs, resolved from config × film overrides × frames. */
    private record ResolvedChain(ModelIKCache.CompiledChain chain, List<String> workIds, Vector3f target, Quaternionf tipTarget, boolean pole, Vector3f polePoint, float poleAngle, float softness, float weight)
    {
    }

    public static void apply(IModel model, List<ModelIKCache.CompiledChain> chains, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides, Map<String, BoneConstraint> boneLimits)
    {
        if (model == null || chains == null || chains.isEmpty())
        {
            return;
        }

        /* Ancestor groups (shallower root) first, and frames re-collected per
         * group, so a child chain (e.g. an arm) sees the pose its parent chain
         * (e.g. the body) already produced and rides along with it. */
        List<ModelIKCache.CompiledChain> ordered = new ArrayList<>(chains);
        ordered.sort(Comparator.comparingInt((ModelIKCache.CompiledChain chain) -> rootDepth(model, chain)));

        if (LOG_IK)
        {
            LOG.setLength(0);
        }

        if (model instanceof Model cubic)
        {
            /* OVERLAPPING chains merge into one tree and solve together — shared
             * bones negotiate between the goals (Blender's tree merge). Disjoint
             * chains stay independent solves, ancestor-first as before. */
            for (List<ModelIKCache.CompiledChain> group : groupOverlapping(ordered))
            {
                Set<String> wanted = new HashSet<>();

                for (ModelIKCache.CompiledChain chain : group)
                {
                    wanted.add(chain.target());
                    wanted.addAll(chain.chainRootToEffector());

                    if (chain.poleTarget() != null && !chain.poleTarget().isEmpty())
                    {
                        wanted.add(chain.poleTarget());
                    }
                }

                Map<String, PivotFrame> frames = new HashMap<>(wanted.size() * 2);

                ModelPivotFrames.collect(model, wanted, frames, null);
                applyGroupCubic(cubic, group, frames, jointDoF, controllerTargets, poleTargets, targetWeights, poleWeights, controlOverrides);
            }
        }
        else if (model instanceof BOBJModel bobj)
        {
            for (ModelIKCache.CompiledChain chain : ordered)
            {
                Set<String> wanted = new HashSet<>();
                wanted.add(chain.target());
                wanted.addAll(chain.chainRootToEffector());

                if (chain.poleTarget() != null && !chain.poleTarget().isEmpty())
                {
                    wanted.add(chain.poleTarget());
                }

                Map<String, PivotFrame> frames = new HashMap<>(wanted.size() * 2);

                ModelPivotFrames.collect(model, wanted, frames, null);

                ResolvedChain resolved = resolveChain(model, chain, frames, controllerTargets, poleTargets, targetWeights, poleWeights, controlOverrides);

                if (resolved != null)
                {
                    applyChainBobjLegacy(bobj, resolved, frames, boneLimits);
                }
            }
        }

        if (LOG_IK)
        {
            flushLog();
        }
    }

    /**
     * Buckets the (ancestor-first ordered) chains into groups of transitively
     * overlapping bone sets. Group order follows the order of each group's
     * first chain, so ancestor groups still apply first.
     */
    private static List<List<ModelIKCache.CompiledChain>> groupOverlapping(List<ModelIKCache.CompiledChain> ordered)
    {
        List<List<ModelIKCache.CompiledChain>> groups = new ArrayList<>();
        List<Set<String>> groupBones = new ArrayList<>();

        for (ModelIKCache.CompiledChain chain : ordered)
        {
            List<Integer> touching = new ArrayList<>();

            for (int g = 0; g < groups.size(); g++)
            {
                for (String bone : chain.chainRootToEffector())
                {
                    if (groupBones.get(g).contains(bone))
                    {
                        touching.add(g);
                        break;
                    }
                }
            }

            if (touching.isEmpty())
            {
                List<ModelIKCache.CompiledChain> group = new ArrayList<>();

                group.add(chain);
                groups.add(group);
                groupBones.add(new HashSet<>(chain.chainRootToEffector()));

                continue;
            }

            /* Merge every touched group into the first one, then add the chain. */
            int first = touching.get(0);

            for (int t = touching.size() - 1; t >= 1; t--)
            {
                int g = touching.get(t);

                groups.get(first).addAll(groups.get(g));
                groupBones.get(first).addAll(groupBones.get(g));
                groups.remove(g);
                groupBones.remove(g);
            }

            groups.get(first).add(chain);
            groupBones.get(first).addAll(chain.chainRootToEffector());
        }

        return groups;
    }

    /** Writes this frame's accumulated solve dump over the previous frame's. */
    private static void flushLog()
    {
        try (java.io.PrintWriter writer = new java.io.PrintWriter("run/ik-log.txt"))
        {
            writer.print(LOG);
        }
        catch (java.io.IOException e)
        {
            e.printStackTrace();
        }
    }

    /** Depth of the chain's root bone from the model root, for ancestor-first ordering. */
    private static int rootDepth(IModel model, ModelIKCache.CompiledChain chain)
    {
        List<String> ids = chain.chainRootToEffector();
        String group = ids.isEmpty() ? chain.tip() : ids.get(0);

        return depthOf(model, group);
    }

    /** Parent-walk depth of a bone from the model root. */
    private static int depthOf(IModel model, String bone)
    {
        String group = bone;
        int depth = 0;

        while (group != null && !group.isEmpty() && depth < 256)
        {
            String parent = model.getParentGroupKey(group);

            if (parent == null || parent.equals(group))
            {
                break;
            }

            group = parent;
            depth++;
        }

        return depth;
    }

    /**
     * Resolves one chain's solve inputs for this frame: the film's `ik` track
     * overrides over the config scalars, the (possibly faded) target and pole
     * positions, the auto-tail work ids and the tail-shifted target. Returns
     * {@code null} when the chain is off this frame (disabled, weightless, or
     * its target frame is missing).
     */
    private static ResolvedChain resolveChain(IModel model, ModelIKCache.CompiledChain chain, Map<String, PivotFrame> frames, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides)
    {
        /* The film's `ik` track may override the chain's static config scalars.
         * IK weight is independent of pose `fix` — freezing a bone pins it to rest
         * (changing the FK pose IK reads from) but no longer gates IK weight, which
         * comes only from the config and the `ik` track. */
        IKControl control = controlOverrides == null ? null : controlOverrides.get(chain.tip());

        if (control != null && !control.enabled)
        {
            return null;
        }

        boolean pole = control != null ? control.pole : chain.pole();
        float softness = control != null ? control.softness : chain.softness();
        float weight = control != null ? control.weight : chain.weight();
        float poleAngle = (float) Math.toRadians(control != null ? control.poleAngle : chain.poleAngle());

        if (weight <= 0F)
        {
            return null;
        }

        PivotFrame targetFrame = frames.get(chain.target());

        if (targetFrame == null)
        {
            return null;
        }

        List<String> chainIds = chain.chainRootToEffector();

        /* Auto-tail (foot IK): with "tip follows target" on, a chain ending in a bare
         * marker bone (no geometry, no children) treats that marker as the EFFECTOR's tail
         * — the bone before it becomes the orientable end, and the IK reaches the tail. So
         * the foot turns to the controller while the leg above bends to plant the tail (the
         * foot's bottom) on the target. Off, or no marker: the chain is used as-is. */
        boolean tipRotation = chain.tipRotation();
        String tailId = tipRotation ? autoTailId(model, chainIds) : null;
        List<String> workIds = tailId == null ? chainIds : chainIds.subList(0, chainIds.size() - 1);

        if (workIds.size() < 2)
        {
            return null;
        }

        /* The film's target/pole overrides ride a 0..1 weight that eases them in/out across
         * a "None" keyframe, so fading a target glides from the bone's own frame, not origin. */
        Vector3f override = controllerTargets == null ? null : controllerTargets.get(chain.target());
        Vector3f target = new Vector3f(targetFrame.position());

        if (override != null)
        {
            target.lerp(override, weightOf(targetWeights, chain.target()));
        }

        /* "Tip follows target": the effector copies the controller's orientation. Null = keep FK. */
        Quaternionf tipTarget = tipRotation && targetFrame.worldRotation() != null ? new Quaternionf(targetFrame.worldRotation()) : null;

        /* Foot IK: back the reach off so the effector's TAIL (the marker), not its pivot,
         * lands on the target once the effector is turned to the controller's orientation. */
        if (tailId != null && tipTarget != null)
        {
            shiftTargetForTail(target, tipTarget, workIds.get(workIds.size() - 1), tailId, frames);
        }

        Vector3f polePoint = resolvePolePoint(pole, chain.poleTarget(), frames, poleTargets, poleWeights);

        return new ResolvedChain(chain, workIds, target, tipTarget, pole, polePoint, poleAngle, softness, weight);
    }

    /* ------------------------------------------------------------------ */
    /* Cubic: channel-space damped-least-squares solve over a merged tree  */
    /* ------------------------------------------------------------------ */

    /**
     * The cubic solve for one group of overlapping chains: capture the union of
     * their directed bones into the channel-space tree ({@link IKTree}), one
     * effector per chain, soften each goal against its own chain's reach, run
     * the DLS solve (with Blender's pole per chain), and write each bone's
     * solved local rotation to {@link ModelGroup#orient} blended against the FK
     * base by the IK weight. The euler channels are never touched — they stay
     * the read-only FK truth (the constraint-stack contract), and the solved
     * angles START from them, so the twist the animator posed survives into
     * the solve by construction.
     */
    private static void applyGroupCubic(Model model, List<ModelIKCache.CompiledChain> group, Map<String, PivotFrame> frames, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides)
    {
        List<ResolvedChain> resolved = new ArrayList<>(group.size());

        for (ModelIKCache.CompiledChain chain : group)
        {
            ResolvedChain r = resolveChain(model, chain, frames, controllerTargets, poleTargets, targetWeights, poleWeights, controlOverrides);

            if (r != null)
            {
                resolved.add(r);
            }
        }

        if (resolved.isEmpty())
        {
            return;
        }

        /* The tree's nodes: the union of every chain's DIRECTED bones (all work
         * ids but the last — the last is the effector bone, whose own angles
         * move only what hangs below it), parents-first. */
        LinkedHashSet<String> nodeSet = new LinkedHashSet<>();

        for (ResolvedChain r : resolved)
        {
            nodeSet.addAll(r.workIds().subList(0, r.workIds().size() - 1));
        }

        List<String> nodes = new ArrayList<>(nodeSet);

        nodes.sort(Comparator.comparingInt((String bone) -> depthOf(model, bone)));

        IKTree tree = new IKTree(nodes.size(), resolved.size());
        Map<String, Integer> nodeIndex = new HashMap<>(nodes.size() * 2);

        for (int i = 0; i < nodes.size(); i++)
        {
            nodeIndex.put(nodes.get(i), i);
        }

        for (int i = 0; i < nodes.size(); i++)
        {
            ModelGroup bone = model.getGroup(nodes.get(i));
            PivotFrame frame = frames.get(nodes.get(i));

            if (bone == null || frame == null)
            {
                return;
            }

            IKJoint joint = tree.joints[i];

            joint.startPosition.set(frame.position());
            joint.startWorldRotation.set(frame.worldRotation());
            tree.startParentRotation[i].set(frame.parentRotation());
            sourceAngles(bone, joint.startAngles);
            joint.angles.set(joint.startAngles);
            tree.parentIndex[i] = nearestAncestor(model, nodes.get(i), nodeIndex);

            ModelIKConfig.JointDoF dof = jointDoF == null ? null : jointDoF.get(nodes.get(i));

            if (dof != null)
            {
                applyDoF(joint, dof);
            }
        }

        /* Effectors: one per chain, riding its last directed bone; each goal is
         * softened against its OWN chain's reach. Poles are per chain too. */
        IKTreeSolver.Pole[] poles = new IKTreeSolver.Pole[resolved.size()];

        for (int e = 0; e < resolved.size(); e++)
        {
            ResolvedChain r = resolved.get(e);
            List<String> workIds = r.workIds();
            PivotFrame effectorFrame = frames.get(workIds.get(workIds.size() - 1));
            Integer lastJoint = nodeIndex.get(workIds.get(workIds.size() - 2));
            Integer rootJoint = nodeIndex.get(workIds.get(0));

            if (effectorFrame == null || lastJoint == null || rootJoint == null)
            {
                return;
            }

            IKTree.Effector effector = tree.effector(e, lastJoint);

            effector.startPosition.set(effectorFrame.position());
            effector.weight = r.weight();

            Vector3f rootPosition = tree.joints[rootJoint].startPosition;
            float reach = chainReach(tree, rootJoint, lastJoint, effector.startPosition);

            effector.goal.set(IKTreeSolver.softGoal(rootPosition, reach, r.target(), r.softness()));

            Vector3f polePoint = r.polePoint();

            if (r.pole() && polePoint == null)
            {
                polePoint = restVirtualPole(model, workIds, tree.startParentRotation[rootJoint], rootPosition, reach);
            }

            poles[e] = polePoint == null ? null : new IKTreeSolver.Pole(rootJoint, polePoint, r.poleAngle());
        }

        if (LOG_IK)
        {
            logTreeIn(nodes, tree);
        }

        IKTreeSolver.Result result = IKTreeSolver.solve(tree, poles, IKTreeSolver.Params.DEFAULT);

        if (LOG_IK)
        {
            logTreeOut(tree, result);
        }

        writeTree(model, nodes, tree, resolved);
    }

    /** The captured arc length root joint → last joint → effector point, along the tree. */
    private static float chainReach(IKTree tree, int rootJoint, int lastJoint, Vector3f effectorStart)
    {
        float total = effectorStart.distance(tree.joints[lastJoint].startPosition);
        int j = lastJoint;

        while (j != rootJoint && tree.parentIndex[j] >= 0)
        {
            int parent = tree.parentIndex[j];

            total += tree.joints[j].startPosition.distance(tree.joints[parent].startPosition);
            j = parent;
        }

        return total;
    }

    /** Index of the bone's nearest ancestor among the tree nodes; -1 when none. */
    private static int nearestAncestor(IModel model, String bone, Map<String, Integer> nodeIndex)
    {
        String group = model.getParentGroupKey(bone);
        int depth = 0;

        while (group != null && !group.isEmpty() && depth < 256)
        {
            Integer index = nodeIndex.get(group);

            if (index != null)
            {
                return index;
            }

            String parent = model.getParentGroupKey(group);

            if (parent == null || parent.equals(group))
            {
                break;
            }

            group = parent;
            depth++;
        }

        return -1;
    }

    /**
     * The bone's FK rotation as ZYX angles in radians — the solve's start value.
     * Euler bones read their channels directly (cubic channels are degrees);
     * quaternion-mode bones and bones carrying a composed {@code orient} (layer
     * stacks) decompose the evaluated rotation compatibly against the channels,
     * so the start stays continuous with what the animator sees.
     */
    private static Vector3f sourceAngles(ModelGroup bone, Vector3f dest)
    {
        float toRad = (float) (Math.PI / 180.0);
        Vector3f channels = dest.set(bone.current.rotate).mul(toRad);

        if (bone.orient == null && bone.current.rotationMode != Transform.RotationMode.QUATERNION)
        {
            return channels;
        }

        return Matrices.toCompatibleEulerZYXRadians(bone.evaluatedRotation(), new Vector3f(channels), dest);
    }

    /** Copies the config's per-bone freedom onto a solver joint; limits are authored in degrees. */
    private static void applyDoF(IKJoint joint, ModelIKConfig.JointDoF dof)
    {
        float toRad = (float) (Math.PI / 180.0);

        joint.locked[0] = dof.lockX();
        joint.locked[1] = dof.lockY();
        joint.locked[2] = dof.lockZ();

        joint.limited[0] = dof.limitX();
        joint.limited[1] = dof.limitY();
        joint.limited[2] = dof.limitZ();

        joint.limitMin[0] = dof.minX() * toRad;
        joint.limitMin[1] = dof.minY() * toRad;
        joint.limitMin[2] = dof.minZ() * toRad;

        joint.limitMax[0] = dof.maxX() * toRad;
        joint.limitMax[1] = dof.maxY() * toRad;
        joint.limitMax[2] = dof.maxZ() * toRad;

        joint.stiffness[0] = dof.stiffnessX();
        joint.stiffness[1] = dof.stiffnessY();
        joint.stiffness[2] = dof.stiffnessZ();
    }

    /**
     * The rest-authored virtual pole point for a chain: the direction its first
     * interior pivot sticks out from the rest root-to-effector line — where the
     * model's own elbow/knee points — lifted into the world by the chain root's
     * current parent frame and placed a reach away from the root. {@code null}
     * when the chain is too short or authored dead straight (no side to prefer).
     */
    private static Vector3f restVirtualPole(Model model, List<String> workIds, Quaternionf rootParentRotation, Vector3f rootPosition, float reach)
    {
        if (workIds.size() < 3)
        {
            return null;
        }

        ModelGroup root = model.getGroup(workIds.get(0));
        ModelGroup elbow = model.getGroup(workIds.get(1));
        ModelGroup effector = model.getGroup(workIds.get(workIds.size() - 1));

        if (root == null || elbow == null || effector == null)
        {
            return null;
        }

        Vector3f axis = new Vector3f(effector.initial.translate).sub(root.initial.translate);

        if (axis.lengthSquared() < EPS * EPS)
        {
            return null;
        }

        axis.normalize();

        Vector3f side = perpendicularTo(new Vector3f(elbow.initial.translate).sub(root.initial.translate), axis);

        if (side == null)
        {
            return null;
        }

        rootParentRotation.transform(side);

        return new Vector3f(rootPosition).fma(reach, side);
    }

    /**
     * Writes the solved tree onto the bones: each node's local rotation is
     * composed from its solved channel angles and written raw to
     * {@link ModelGroup#orient}, blended against the FK base (the bone's
     * evaluated rotation) by the strongest weight of the chains running through
     * it. The blended world frames advance the same rigid way the solve did —
     * the frames the renderer establishes — so each chain's tip target lands in
     * the right frame at any weight.
     */
    private static void writeTree(Model model, List<String> nodes, IKTree tree, List<ResolvedChain> resolved)
    {
        int n = nodes.size();
        float[] nodeWeight = new float[n];

        for (int e = 0; e < resolved.size(); e++)
        {
            float weight = resolved.get(e).weight();

            for (int i = 0; i < n; i++)
            {
                if (tree.moves(e, i))
                {
                    nodeWeight[i] = Math.max(nodeWeight[i], weight);
                }
            }
        }

        Quaternionf[] blendedWorld = new Quaternionf[n];

        for (int i = 0; i < n; i++)
        {
            ModelGroup bone = model.getGroup(nodes.get(i));

            if (bone == null)
            {
                return;
            }

            Quaternionf solvedLocal = Matrices.toLocalRotationZYXRadians(tree.joints[i].angles);
            Quaternionf applied = nodeWeight[i] >= 1F - EPS ? solvedLocal : bone.evaluatedRotation().slerp(solvedLocal, nodeWeight[i]);

            bone.orient = applied;

            /* Blended world walk, rigid-model style: the parent frame is the captured
             * one carried by how far the nearest captured ancestor's BLENDED world
             * rotation moved — the frame the renderer will actually establish. */
            int parent = tree.parentIndex[i];
            Quaternionf blendedParent;

            if (parent < 0)
            {
                blendedParent = new Quaternionf(tree.startParentRotation[i]);
            }
            else
            {
                Quaternionf delta = new Quaternionf(blendedWorld[parent]).mul(new Quaternionf(tree.joints[parent].startWorldRotation).conjugate());

                blendedParent = delta.mul(tree.startParentRotation[i]);
            }

            blendedWorld[i] = blendedParent.mul(applied, new Quaternionf());
        }

        /* Tip follows target: each chain's effector bone (not a solver node of its
         * own chain) copies the controller's world orientation, in its parent's
         * BLENDED frame. Written after the nodes, so on the rare rig where a tip
         * doubles as another chain's directed bone, the tip orientation wins. */
        for (int e = 0; e < resolved.size(); e++)
        {
            ResolvedChain r = resolved.get(e);

            if (r.tipTarget() == null)
            {
                continue;
            }

            List<String> workIds = r.workIds();
            ModelGroup tip = model.getGroup(workIds.get(workIds.size() - 1));
            Integer lastJoint = null;

            for (int i = 0; i < n; i++)
            {
                if (nodes.get(i).equals(workIds.get(workIds.size() - 2)))
                {
                    lastJoint = i;
                    break;
                }
            }

            if (tip == null || lastJoint == null)
            {
                continue;
            }

            Quaternionf tipLocal = new Quaternionf(blendedWorld[lastJoint]).conjugate().mul(r.tipTarget());

            tip.orient = r.weight() >= 1F - EPS ? tipLocal : tip.evaluatedRotation().slerp(tipLocal, r.weight());
        }
    }

    private static void logTreeIn(List<String> nodes, IKTree tree)
    {
        LOG.append("tree ").append(nodes).append('\n');

        for (int e = 0; e < tree.effectors.length; e++)
        {
            IKTree.Effector effector = tree.effectors[e];

            LOG.append("  goal[").append(e).append("] ").append(fmt(effector.goal))
                .append(" weight ").append(effector.weight)
                .append(" rides ").append(nodes.get(effector.joint)).append('\n');
        }

        for (int i = 0; i < tree.joints.length; i++)
        {
            LOG.append("  in  ").append(nodes.get(i)).append(" angles ").append(fmtDeg(tree.joints[i].angles))
                .append(" pos ").append(fmt(tree.joints[i].startPosition)).append('\n');
        }
    }

    private static void logTreeOut(IKTree tree, IKTreeSolver.Result result)
    {
        LOG.append("  solved reached=").append(result.reached()).append(" err=").append(result.error())
            .append(" iters=").append(result.iterations()).append('\n');

        for (IKJoint joint : tree.joints)
        {
            LOG.append("  out angles ").append(fmtDeg(joint.angles)).append('\n');
        }
    }

    private static String fmt(Vector3f v)
    {
        return String.format("(%+.4f, %+.4f, %+.4f)", v.x, v.y, v.z);
    }

    private static String fmtDeg(Vector3f radians)
    {
        float toDeg = (float) (180.0 / Math.PI);

        return String.format("(%+.2f, %+.2f, %+.2f)", radians.x * toDeg, radians.y * toDeg, radians.z * toDeg);
    }

    /**
     * The chain's trailing tail marker: a bare cubic end bone — no geometry, no children —
     * that stands in for the effector's tail (the reach point a pivot-based bone can't
     * express itself). Returns its id so the bone before it becomes the orientable end;
     * {@code null} when there is none, the model is not cubic, or the chain is too short to
     * keep a bendable run after dropping the tail.
     */
    private static String autoTailId(IModel model, List<String> chainIds)
    {
        if (chainIds.size() < 4 || !(model instanceof Model cubic))
        {
            return null;
        }

        String lastId = chainIds.get(chainIds.size() - 1);
        ModelGroup last = cubic.getGroup(lastId);

        if (last == null || !last.cubes.isEmpty() || !last.meshes.isEmpty() || !last.children.isEmpty())
        {
            return null;
        }

        return lastId;
    }

    /**
     * Backs the IK target off by the effector's tail offset, turned to the controller's
     * orientation, so the tail (the marker) — not the effector's pivot — lands on the
     * original target once the effector is oriented to the controller. The offset is the
     * tail's rest position in the effector's local frame (constant geometry).
     */
    private static void shiftTargetForTail(Vector3f target, Quaternionf tipTarget, String effectorId, String tailId, Map<String, PivotFrame> frames)
    {
        PivotFrame eff = frames.get(effectorId);
        PivotFrame tail = frames.get(tailId);

        if (eff == null || tail == null || eff.worldRotation() == null)
        {
            return;
        }

        Vector3f offsetLocal = new Quaternionf(eff.worldRotation()).conjugate().transform(new Vector3f(tail.position()).sub(eff.position()));
        Vector3f shift = new Quaternionf(tipTarget).transform(offsetLocal);

        target.sub(shift);
    }

    /**
     * Resolves the pole target into a model-space point the bend aims at: the
     * film override position if the chain's pole bone is being driven, otherwise
     * the pole bone's current position. Returns {@code null} (no pole
     * constraint) when the chain has no pole or no pole target.
     */
    private static Vector3f resolvePolePoint(boolean pole, String poleTarget, Map<String, PivotFrame> frames, Map<String, Vector3f> poleTargets, Map<String, Float> poleWeights)
    {
        if (!pole || poleTarget == null || poleTarget.isEmpty())
        {
            return null;
        }

        Vector3f override = poleTargets == null ? null : poleTargets.get(poleTarget);
        PivotFrame frame = frames.get(poleTarget);
        Vector3f config = frame == null ? null : new Vector3f(frame.position());

        if (override == null)
        {
            return config;
        }

        /* Slide the pole from its config bone to the keyframed target by the fade
         * weight, so fading a pole in/out glides from the config pole, not origin. */
        return config == null ? new Vector3f(override) : config.lerp(override, weightOf(poleWeights, poleTarget));
    }

    /** The override's 0..1 fade weight (1 = full override) — 1 when the chain has no weighted fade this frame. */
    private static float weightOf(Map<String, Float> weights, String id)
    {
        return weights == null ? 1F : weights.getOrDefault(id, 1F);
    }

    /**
     * The chain's authored bend side, in model space: the normal of the rest bend
     * plane {@code restDir[0] x restDir[1]} — the side the limb was modelled bent
     * towards (knee forward, elbow back) — taken in the root's local frame and
     * lifted by the root's current world rotation, so it tracks the limb as the
     * shoulder/hip turns. Returns {@code null} when the chain is shorter than two
     * bones or the rest pose is dead straight (no authored bend — a rope).
     */
    private static Vector3f restBendNormal(IModel model, List<String> chainIds, Quaternionf rootParentRotation)
    {
        if (chainIds.size() < 3)
        {
            return null;
        }

        Vector3f a = restDirection(model, chainIds, 0);
        Vector3f b = restDirection(model, chainIds, 1);

        if (a == null || b == null)
        {
            return null;
        }

        Vector3f normal = new Vector3f(a).cross(b);

        if (normal.lengthSquared() < EPS * EPS)
        {
            return null;
        }

        return rootParentRotation.transform(normal.normalize());
    }

    /**
     * The bone's local rest direction towards its child, taken exactly as that
     * model's renderer takes it (cubic: pivot difference; BOBJ: the renderer's
     * own {@link ModelRotationBlender#getBobjRestDirection}).
     */
    private static Vector3f restDirection(IModel model, List<String> chainIds, int i)
    {
        String id = chainIds.get(i);
        String childId = chainIds.get(i + 1);

        if (model instanceof Model cubic)
        {
            ModelGroup bone = cubic.getGroup(id);
            ModelGroup child = cubic.getGroup(childId);

            if (bone == null || child == null)
            {
                return null;
            }

            return normalizeRest(new Vector3f(child.initial.translate).sub(bone.initial.translate));
        }

        if (model instanceof BOBJModel bobj)
        {
            BOBJBone bone = bobj.getArmature().bones.get(id);
            BOBJBone child = bobj.getArmature().bones.get(childId);

            if (bone == null)
            {
                return null;
            }

            return normalizeRest(ModelRotationBlender.getBobjRestDirection(bobj, bone, child, chainIds, i));
        }

        return null;
    }

    private static Vector3f normalizeRest(Vector3f restDir)
    {
        if (restDir.lengthSquared() < 1.0e-12f)
        {
            restDir.set(0F, -1F, 0F);
        }

        restDir.normalize();

        return restDir;
    }

    /** {@code v} projected onto the plane perpendicular to unit {@code axis}, normalized; {@code null} if degenerate. */
    private static Vector3f perpendicularTo(Vector3f v, Vector3f axis)
    {
        Vector3f out = new Vector3f(v);
        float dot = out.dot(axis);

        out.x -= axis.x * dot;
        out.y -= axis.y * dot;
        out.z -= axis.z * dot;

        return out.lengthSquared() < EPS * EPS ? null : out.normalize();
    }

    /* ------------------------------------------------------------------ */
    /* LEGACY BOBJ PATH — the old position-level solve, kept verbatim      */
    /* until the BOBJ port onto the channel-space core. Everything below   */
    /* serves only BOBJ models and goes away with that port.               */
    /* ------------------------------------------------------------------ */

    private static void applyChainBobjLegacy(BOBJModel model, ResolvedChain resolved, Map<String, PivotFrame> frames, Map<String, BoneConstraint> boneLimits)
    {
        List<String> workIds = resolved.workIds();
        List<Vector3f> currentPositions = new ArrayList<>(workIds.size());
        Quaternionf rootParentRotation = null;

        for (String id : workIds)
        {
            PivotFrame frame = frames.get(id);

            if (frame == null)
            {
                return;
            }

            currentPositions.add(new Vector3f(frame.position()));

            if (rootParentRotation == null)
            {
                rootParentRotation = new Quaternionf(frame.parentRotation());
            }
        }

        if (rootParentRotation == null)
        {
            return;
        }

        IKSolver.Limit[] limits = buildLimits(model, workIds, boneLimits);
        Vector3f restHinge = restBendNormal(model, workIds, rootParentRotation);

        Vector3f bendNormal = new Vector3f();
        List<Vector3f> solved = IKSolver.solve(currentPositions, resolved.target(), resolved.pole(), resolved.polePoint(), resolved.poleAngle(), resolved.softness(), LEGACY_MAX_ITERATIONS, LEGACY_TOLERANCE, limits, limits == null ? null : rootParentRotation, restHinge, bendNormal);

        /* The bend-plane normal the solve settled on (zero when undefined). Passed to the
         * orientation pass as the roll-reference seed so a fully straight chain keeps a
         * stable twist instead of jittering at the reach boundary. */
        Vector3f bendSeed = bendNormal.lengthSquared() < EPS * EPS ? null : bendNormal;

        if (workIds.size() >= 3)
        {
            buildChainOrientationsBobj(model, workIds, solved, rootParentRotation, resolved.weight(), resolved.tipTarget(), bendSeed);
        }
        else
        {
            Vector3f[] solvedArray = solved.toArray(new Vector3f[solved.size()]);
            ModelRotationBlender.applyWeightedRotations(model, rootParentRotation, workIds, solvedArray, resolved.weight());
        }
    }

    /**
     * Gives each BOBJ IK bone a full local orientation written to
     * {@link BOBJBone#orient}, which the armature applies in place of the euler
     * rotate. BOBJ bones carry a per-bone REST rotation (their {@code relBoneMat}),
     * so the rest and solved frames are walked separately: the rest frame advances
     * by {@code relBoneMat} alone, the solved frame by each bone's applied
     * orientation then {@code relBoneMat}. Both build the roll reference by
     * parallel transport in world, so at rest the two frames coincide and the
     * orientation is identity — no baseline twist. Cubic's X-mirror convention
     * ({@link Matrices#orientMirroredX}) applies to the shared model space.
     */
    private static void buildChainOrientationsBobj(BOBJModel model, List<String> chainIds, List<Vector3f> solved, Quaternionf rootParentRotation, float weight, Quaternionf tipTarget, Vector3f bendSeed)
    {
        int bones = chainIds.size() - 1;
        Map<String, BOBJBone> bonesMap = model.getArmature().bones;
        BOBJBone[] chainBones = new BOBJBone[bones];
        Vector3f[] restDir = new Vector3f[bones];
        Quaternionf[] relRot = new Quaternionf[bones];
        Vector3f[] segWorld = new Vector3f[bones];

        for (int i = 0; i < bones; i++)
        {
            BOBJBone bone = bonesMap.get(chainIds.get(i));
            Vector3f seg = new Vector3f(solved.get(i + 1)).sub(solved.get(i));

            restDir[i] = restDirection(model, chainIds, i);

            if (bone == null || restDir[i] == null || seg.lengthSquared() < EPS * EPS)
            {
                return;
            }

            chainBones[i] = bone;
            relRot[i] = bone.relBoneMat.getNormalizedRotation(new Quaternionf());
            segWorld[i] = seg.normalize();
        }

        /* Rest-pose world frames advance by relBoneMat alone (geometry rest, no bone
         * rotation); the root's own relBoneMat is already baked into rootParentRotation. */
        Quaternionf[] restFrame = new Quaternionf[bones];
        restFrame[0] = new Quaternionf(rootParentRotation);

        for (int i = 1; i < bones; i++)
        {
            restFrame[i] = new Quaternionf(restFrame[i - 1]).mul(relRot[i]);
        }

        Vector3f[] restDirWorld = new Vector3f[bones];

        for (int i = 0; i < bones; i++)
        {
            restDirWorld[i] = restFrame[i].transform(new Vector3f(restDir[i]));
        }

        Vector3f[] restNormalWorld = transportNormals(restDirWorld);
        Vector3f[] solvedNormalWorld = transportNormals(segWorld, bendSeed);

        /* Solved-pose world frame advances by each bone's applied orientation, then the
         * next bone's relBoneMat — so a child decomposes against the frame the armature
         * actually establishes (blended orientation at weight < 1). */
        Quaternionf originRot = new Quaternionf(rootParentRotation);

        for (int i = 0; i < bones; i++)
        {
            Quaternionf invOrigin = new Quaternionf(originRot).conjugate();
            Vector3f segLocal = invOrigin.transform(new Vector3f(segWorld[i]));
            Vector3f normalLocal = invOrigin.transform(new Vector3f(solvedNormalWorld[i]));
            Vector3f restNormalLocal = new Quaternionf(restFrame[i]).conjugate().transform(new Vector3f(restNormalWorld[i]));

            Quaternionf localRot = Matrices.orientMirroredX(restDir[i], restNormalLocal, segLocal, normalLocal);
            Quaternionf oriented = weight >= 1F - EPS ? new Quaternionf(localRot) : chainBones[i].evaluatedRotation().slerp(localRot, weight);

            chainBones[i].orient = oriented;

            if (i + 1 < bones)
            {
                originRot.mul(oriented).mul(relRot[i + 1]);
            }
        }

        /* Tip follows target: the effector copies the controller's world orientation. Its
         * parent frame is the last directed bone's frame advanced by its applied
         * orientation and the tip's own relBoneMat. */
        if (tipTarget != null)
        {
            BOBJBone tip = bonesMap.get(chainIds.get(chainIds.size() - 1));

            if (tip != null)
            {
                Quaternionf tipRelRot = tip.relBoneMat.getNormalizedRotation(new Quaternionf());
                Quaternionf tipParent = new Quaternionf(originRot).mul(chainBones[bones - 1].orient).mul(tipRelRot);
                Quaternionf tipLocal = tipParent.conjugate().mul(tipTarget);

                tip.orient = weight >= 1F - EPS ? new Quaternionf(tipLocal) : tip.evaluatedRotation().slerp(tipLocal, weight);
            }
        }
    }

    /**
     * Carries a roll-reference normal along a chain of unit directions by parallel
     * transport: seeded from the bend of the first two segments (a stable perpendicular
     * when they are collinear), then rotated minimally from each segment to the next.
     */
    private static Vector3f[] transportNormals(Vector3f[] dirs)
    {
        return transportNormals(dirs, null);
    }

    /**
     * {@code seedHint} (when non-null) is a stable bend-plane normal used to seed the
     * roll reference where the first two segments are collinear — a straightened chain.
     * It is the LIMIT the live bend normal {@code dirs[0] x dirs[1]} approaches as the
     * chain straightens, so seeding with it keeps the roll continuous through full
     * extension.
     */
    private static Vector3f[] transportNormals(Vector3f[] dirs, Vector3f seedHint)
    {
        int m = dirs.length;
        Vector3f[] normals = new Vector3f[m];
        Vector3f seed = m >= 2 ? new Vector3f(dirs[0]).cross(dirs[1]) : new Vector3f();

        if (seed.lengthSquared() < 1.0e-10f)
        {
            Vector3f hint = seedHint == null ? null : perpendicularTo(seedHint, dirs[0]);

            normals[0] = hint != null ? hint : stablePerpendicular(dirs[0]);
        }
        else
        {
            normals[0] = seed.normalize();
        }

        for (int i = 1; i < m; i++)
        {
            Vector3f n = new Quaternionf().rotationTo(dirs[i - 1], dirs[i]).transform(new Vector3f(normals[i - 1]));

            normals[i] = n.normalize();
        }

        return normals;
    }

    /** A deterministic unit perpendicular to {@code dir}, cross with world Z (falling back to world Y when parallel). */
    private static Vector3f stablePerpendicular(Vector3f dir)
    {
        Vector3f perp = new Vector3f(dir).cross(0F, 0F, 1F);

        if (perp.lengthSquared() < EPS * EPS)
        {
            perp.set(dir).cross(0F, 1F, 0F);
        }

        return perp.normalize();
    }

    /**
     * Builds per-bone rotation limits for the legacy BOBJ solve from the
     * constraints config (the cubic path's limits live in the IK config's
     * per-bone DoF instead). Returns {@code null} when no bone in the chain is
     * constrained (fast path).
     */
    private static IKSolver.Limit[] buildLimits(IModel model, List<String> chainIds, Map<String, BoneConstraint> boneLimits)
    {
        if (boneLimits == null || boneLimits.isEmpty())
        {
            return null;
        }

        int directed = chainIds.size() - 1;

        if (directed < 1)
        {
            return null;
        }

        boolean any = false;

        for (int i = 0; i < directed; i++)
        {
            BoneConstraint c = boneLimits.get(chainIds.get(i));

            if (c != null && c.enabled())
            {
                any = true;
                break;
            }
        }

        if (!any)
        {
            return null;
        }

        IKSolver.Limit[] limits = new IKSolver.Limit[directed];

        for (int i = 0; i < directed; i++)
        {
            String id = chainIds.get(i);
            Vector3f restDir = restDirection(model, chainIds, i);

            if (restDir == null)
            {
                return null;
            }

            BoneConstraint c = boneLimits.get(id);
            boolean enabled = c != null && c.enabled();

            limits[i] = enabled
                ? new IKSolver.Limit(true, restDir, c.minX(), c.minY(), c.minZ(), c.maxX(), c.maxY(), c.maxZ())
                : new IKSolver.Limit(false, restDir, 0F, 0F, 0F, 0F, 0F, 0F);
        }

        return limits;
    }
}
