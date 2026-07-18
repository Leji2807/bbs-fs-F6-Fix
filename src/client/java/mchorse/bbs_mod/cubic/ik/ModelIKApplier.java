package mchorse.bbs_mod.cubic.ik;

import mchorse.bbs_mod.bobj.BOBJBone;
import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.cubic.data.model.Model;
import mchorse.bbs_mod.cubic.data.model.ModelGroup;
import mchorse.bbs_mod.cubic.ik.solver.IKJoint;
import mchorse.bbs_mod.cubic.ik.solver.IKTree;
import mchorse.bbs_mod.cubic.ik.solver.IKTreeSolver;
import mchorse.bbs_mod.cubic.model.bobj.BOBJModel;
import mchorse.bbs_mod.cubic.render.CubicRenderer.PivotFrame;
import mchorse.bbs_mod.cubic.render.ModelPivotFrames;
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

/**
 * Directs the compiled IK chains onto a model, cubic and BOBJ alike, through
 * the channel-space tree solver: overlapping chains merge and negotiate,
 * disjoint chains solve independently ancestor-first. Both bone flavours meet
 * the same frame convention — a pivot frame's parentRotation carries
 * everything before the bone's own channel rotation (for BOBJ that includes
 * its rest {@code relBoneMat}), so {@code worldRot = parentRot · Rz·Ry·Rx} and
 * the solver never needs to know which model it is posing. What differs is
 * only reads and writes: cubic channels are degrees on {@link ModelGroup},
 * BOBJ channels are radians on {@link BOBJBone} — both write the result to
 * their {@code orient} (the constraint-stack contract; channels stay the
 * read-only FK truth).
 */
final class ModelIKApplier
{
    /**
     * Per-frame solve dump to {@code run/ik-log.txt} — the IK counterpart of the
     * drag log, and like it the thing to ask for when a solve is disputed:
     * captured angles in, goals, iterations, errors, angles out. Flip to enable;
     * zero cost off.
     *
     * <p>Frames ACCUMULATE, numbered, up to {@link #LOG_FRAMES}: a jitter only
     * shows as a discontinuity BETWEEN consecutive frames, so a single kept
     * frame cannot show it. The cap keeps a forgotten flag from growing the file
     * without bound — roughly half a minute of play, after which logging stops
     * and says so.
     */
    private static final boolean LOG_IK = true;

    /** How many frames the accumulating log keeps before it stops writing. */
    private static final int LOG_FRAMES = 2000;

    /**
     * File in the game folder, like the drag log's — the game RUNS in {@code
     * run/}, so the name must be bare. It read {@code "run/ik-log.txt"} from the
     * day it was written, which resolved to a non-existent {@code run/run/} and
     * threw every frame: the flag had never actually produced a log.
     */
    private static final String LOG_FILE = "ik-log.txt";

    private static final StringBuilder LOG = new StringBuilder();

    private static int logFrame;

    private static final float EPS = 1.0e-6f;

    private ModelIKApplier()
    {
    }

    /** A chain's per-frame solve inputs, resolved from config × film overrides × frames. */
    private record ResolvedChain(ModelIKCache.CompiledChain chain, List<String> workIds, Vector3f target, Quaternionf tipTarget, boolean pole, Vector3f polePoint, float poleAngle, float softness, float weight)
    {
    }

    public static void apply(IModel model, List<ModelIKCache.CompiledChain> chains, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides)
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

        if (LOG_IK && logFrame <= LOG_FRAMES)
        {
            LOG.append("--- frame ").append(logFrame).append(" ---\n");
        }

        /* OVERLAPPING chains merge into one tree and solve together — shared
         * bones negotiate between the goals (Blender's tree merge). Disjoint
         * chains stay independent solves. */
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
            applyGroup(model, group, frames, jointDoF, controllerTargets, poleTargets, targetWeights, poleWeights, controlOverrides);
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

    /**
     * Appends this frame's solve dump, so consecutive frames can be compared —
     * which is the only way a jitter is visible. Stops after {@link #LOG_FRAMES}
     * and leaves a line saying so, so a full log is never mistaken for a short
     * session.
     */
    private static void flushLog()
    {
        if (logFrame > LOG_FRAMES)
        {
            return;
        }

        boolean last = logFrame == LOG_FRAMES;

        if (last)
        {
            LOG.append("--- log full at ").append(LOG_FRAMES).append(" frames, logging stops here ---\n");
        }

        try (java.io.FileWriter writer = new java.io.FileWriter(LOG_FILE, logFrame > 0))
        {
            writer.write(LOG.toString());
        }
        catch (java.io.IOException e)
        {
            e.printStackTrace();
        }

        LOG.setLength(0);
        logFrame++;
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
    /* The channel-space damped-least-squares solve over a merged tree     */
    /* ------------------------------------------------------------------ */

    /**
     * The solve for one group of overlapping chains: capture the union of
     * their directed bones into the channel-space tree ({@link IKTree}), one
     * effector per chain, soften each goal against its own chain's reach, run
     * the DLS solve (with Blender's pole per chain and the tip orientation
     * task), and write each bone's solved local rotation to its {@code orient}
     * blended against the FK base by the IK weight. The channels are never
     * touched — they stay the read-only FK truth (the constraint-stack
     * contract), and the solved angles START from them, so the twist the
     * animator posed survives into the solve by construction.
     */
    private static void applyGroup(IModel model, List<ModelIKCache.CompiledChain> group, Map<String, PivotFrame> frames, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides)
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
            PivotFrame frame = frames.get(nodes.get(i));

            if (frame == null)
            {
                return;
            }

            IKJoint joint = tree.joints[i];

            joint.startPosition.set(frame.position());
            joint.startWorldRotation.set(frame.worldRotation());
            tree.startParentRotation[i].set(frame.parentRotation());

            if (sourceAngles(model, nodes.get(i), joint.startAngles) == null)
            {
                return;
            }

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

            /* Tip follows target, in-solver half: ask the chain to orient its LAST
             * directed bone so the tip, keeping its natural FK local pose, would
             * already face the controller — the chain shares the turn instead of
             * the wrist absorbing all of it. The exact tip snap after the solve
             * (writeTree) then has almost nothing left to correct. One radian of
             * orientation error is worth reach/π length units, so a half turn
             * weighs about as much as a full-reach position miss. */
            if (r.tipTarget() != null)
            {
                Quaternionf tipLocal = evaluatedRotation(model, workIds.get(workIds.size() - 1));

                if (tipLocal != null)
                {
                    effector.orientGoal = new Quaternionf(r.tipTarget()).mul(tipLocal.conjugate());
                    effector.orientWeight = reach / (float) Math.PI;
                }
            }

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

        writeTree(model, nodes, tree, resolved, frames);
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

    /* ------------------------------------------------------------------ */
    /* The two bone flavours behind one door                               */
    /* ------------------------------------------------------------------ */

    /**
     * The bone's FK rotation as ZYX angles in radians — the solve's start value.
     * Euler bones read their channels directly (cubic channels are degrees,
     * BOBJ channels are radians); quaternion-mode bones and bones carrying a
     * composed {@code orient} (layer stacks) decompose the evaluated rotation
     * compatibly against the channels, so the start stays continuous with what
     * the animator sees. {@code null} when the bone does not exist.
     */
    private static Vector3f sourceAngles(IModel model, String id, Vector3f dest)
    {
        if (model instanceof Model cubic)
        {
            ModelGroup bone = cubic.getGroup(id);

            if (bone == null)
            {
                return null;
            }

            float toRad = (float) (Math.PI / 180.0);
            Vector3f channels = dest.set(bone.current.rotate).mul(toRad);

            if (bone.orient == null && bone.current.rotationMode != Transform.RotationMode.QUATERNION)
            {
                return channels;
            }

            return Matrices.toCompatibleEulerZYXRadians(bone.evaluatedRotation(), new Vector3f(channels), dest);
        }

        if (model instanceof BOBJModel bobj)
        {
            BOBJBone bone = bobj.getArmature().bones.get(id);

            if (bone == null)
            {
                return null;
            }

            Vector3f channels = dest.set(bone.transform.rotate);

            if (bone.orient == null && bone.transform.rotationMode != Transform.RotationMode.QUATERNION)
            {
                return channels;
            }

            return Matrices.toCompatibleEulerZYXRadians(bone.evaluatedRotation(), new Vector3f(channels), dest);
        }

        return null;
    }

    /** The bone's evaluated FK local rotation (fresh instance); {@code null} when it does not exist. */
    private static Quaternionf evaluatedRotation(IModel model, String id)
    {
        if (model instanceof Model cubic)
        {
            ModelGroup bone = cubic.getGroup(id);

            return bone == null ? null : bone.evaluatedRotation();
        }

        if (model instanceof BOBJModel bobj)
        {
            BOBJBone bone = bobj.getArmature().bones.get(id);

            return bone == null ? null : bone.evaluatedRotation();
        }

        return null;
    }

    /** Writes the solved local rotation to the bone's {@code orient}; false when it does not exist. */
    private static boolean writeOrient(IModel model, String id, Quaternionf orient)
    {
        if (model instanceof Model cubic)
        {
            ModelGroup bone = cubic.getGroup(id);

            if (bone == null)
            {
                return false;
            }

            bone.orient = orient;

            return true;
        }

        if (model instanceof BOBJModel bobj)
        {
            BOBJBone bone = bobj.getArmature().bones.get(id);

            if (bone == null)
            {
                return false;
            }

            bone.orient = orient;

            return true;
        }

        return false;
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
     * model's own elbow/knee points — lifted into the world and placed a reach
     * away from the root. Cubic rest geometry lives in the authored pivots
     * (lifted by the chain root's current parent frame); BOBJ rest geometry
     * lives in the bind matrices, and the lift is the DELTA of the root's
     * parent frame from its bind orientation (BOBJ ancestors carry authored
     * rest rotations, so the raw parent frame alone would double-count them).
     * {@code null} when the chain is too short or authored dead straight (no
     * side to prefer).
     */
    private static Vector3f restVirtualPole(IModel model, List<String> workIds, Quaternionf rootParentRotation, Vector3f rootPosition, float reach)
    {
        if (workIds.size() < 3)
        {
            return null;
        }

        Vector3f restRoot;
        Vector3f restElbow;
        Vector3f restEffector;
        Quaternionf lift;

        if (model instanceof Model cubic)
        {
            ModelGroup root = cubic.getGroup(workIds.get(0));
            ModelGroup elbow = cubic.getGroup(workIds.get(1));
            ModelGroup effector = cubic.getGroup(workIds.get(workIds.size() - 1));

            if (root == null || elbow == null || effector == null)
            {
                return null;
            }

            restRoot = root.initial.translate;
            restElbow = elbow.initial.translate;
            restEffector = effector.initial.translate;

            /* The rest bend direction lives in the authored pivots — absolute model
             * coordinates (each cubic pivot is placed in model space). To carry it
             * into the current pose it must ride the DELTA of the root's parent frame
             * from its REST orientation, exactly as the BOBJ branch subtracts the
             * bind frame below: lifting by the raw current parent frame would
             * double-count any rest rotation the chain's ancestors carry, tilting
             * the auto-pole even when the model sits in its rest pose. */
            Quaternionf restParent = cubicRestParentRotation(cubic, workIds.get(0));

            lift = new Quaternionf(rootParentRotation).mul(restParent.conjugate());
        }
        else if (model instanceof BOBJModel bobj)
        {
            Map<String, BOBJBone> bones = bobj.getArmature().bones;
            BOBJBone root = bones.get(workIds.get(0));
            BOBJBone elbow = bones.get(workIds.get(1));
            BOBJBone effector = bones.get(workIds.get(workIds.size() - 1));

            if (root == null || elbow == null || effector == null)
            {
                return null;
            }

            restRoot = root.boneMat.getTranslation(new Vector3f());
            restElbow = elbow.boneMat.getTranslation(new Vector3f());
            restEffector = effector.boneMat.getTranslation(new Vector3f());

            /* In bind pose (zero channels) the root's parent frame IS its bind
             * rotation, so the current-vs-bind delta is the exact world lift. */
            Quaternionf bindParent = root.boneMat.getUnnormalizedRotation(new Quaternionf());

            lift = new Quaternionf(rootParentRotation).mul(bindParent.conjugate());
        }
        else
        {
            return null;
        }

        Vector3f axis = new Vector3f(restEffector).sub(restRoot);

        if (axis.lengthSquared() < EPS * EPS)
        {
            return null;
        }

        axis.normalize();

        Vector3f side = perpendicularTo(new Vector3f(restElbow).sub(restRoot), axis);
        // (axis and side are in absolute model rest space; `lift` folds them into the current pose.)

        if (side == null)
        {
            return null;
        }

        lift.transform(side);

        return new Vector3f(rootPosition).fma(reach, side);
    }

    /**
     * The rest-pose world rotation of the chain root's PARENT — the product of
     * every ancestor's authored rest rotation ({@code initial.rotate}, degrees,
     * ZYX), in the same unmirrored model space {@link ModelPivotFrames} captures.
     * Identity when the root sits at the model top or its ancestors carry no rest
     * rotation (the common case, where the auto-pole lift is unchanged). This is
     * the cubic counterpart of BOBJ's bind parent frame.
     */
    private static Quaternionf cubicRestParentRotation(Model cubic, String rootId)
    {
        Quaternionf rest = new Quaternionf();
        String id = cubic.getParentGroupKey(rootId);
        int guard = 0;

        while (id != null && !id.isEmpty() && guard++ < 256)
        {
            ModelGroup bone = cubic.getGroup(id);

            if (bone == null)
            {
                break;
            }

            /* World order is root-most first; walking up, each ancestor pre-multiplies. */
            rest = Matrices.toLocalRotationZYXDegrees(bone.initial.rotate).mul(rest);

            String parent = cubic.getParentGroupKey(id);

            if (parent == null || parent.equals(id))
            {
                break;
            }

            id = parent;
        }

        return rest;
    }

    /**
     * Writes the solved tree onto the bones: each node's local rotation is
     * composed from its solved channel angles and written raw to its
     * {@code orient}, blended against the FK base (the bone's evaluated
     * rotation) by the strongest weight of the chains running through it. The
     * blended world frames advance the same rigid way the solve did — the
     * frames the renderer establishes — so each chain's tip target lands in
     * the right frame at any weight.
     */
    private static void writeTree(IModel model, List<String> nodes, IKTree tree, List<ResolvedChain> resolved, Map<String, PivotFrame> frames)
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
        Quaternionf[] blendedParentOf = new Quaternionf[n];

        for (int i = 0; i < n; i++)
        {
            Quaternionf evaluated = evaluatedRotation(model, nodes.get(i));

            if (evaluated == null)
            {
                return;
            }

            Quaternionf solvedLocal = Matrices.toLocalRotationZYXRadians(tree.joints[i].angles);
            Quaternionf applied = nodeWeight[i] >= 1F - EPS ? solvedLocal : evaluated.slerp(solvedLocal, nodeWeight[i]);

            if (!writeOrient(model, nodes.get(i), applied))
            {
                return;
            }

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

            blendedParentOf[i] = new Quaternionf(blendedParent);
            blendedWorld[i] = blendedParent.mul(applied, new Quaternionf());
        }

        /* Tip follows target: each chain's effector bone (not a solver node of its
         * own chain) copies the controller's world orientation, in its parent's
         * BLENDED frame. The tip's captured parent frame carries everything between
         * the last directed bone and the tip's own rotation (for BOBJ its rest
         * relBoneMat), so it is advanced by the last bone's blended delta — for a
         * cubic bone that reduces to the last bone's blended world exactly. Written
         * after the nodes, so on the rare rig where a tip doubles as another
         * chain's directed bone, the tip orientation wins. */
        for (int e = 0; e < resolved.size(); e++)
        {
            ResolvedChain r = resolved.get(e);

            if (r.tipTarget() == null)
            {
                continue;
            }

            List<String> workIds = r.workIds();
            String tipId = workIds.get(workIds.size() - 1);
            PivotFrame tipFrame = frames.get(tipId);
            Integer lastJoint = null;

            for (int i = 0; i < n; i++)
            {
                if (nodes.get(i).equals(workIds.get(workIds.size() - 2)))
                {
                    lastJoint = i;
                    break;
                }
            }

            if (tipFrame == null || lastJoint == null)
            {
                continue;
            }

            Quaternionf lastDelta = new Quaternionf(blendedWorld[lastJoint]).mul(new Quaternionf(tree.joints[lastJoint].startWorldRotation).conjugate());
            Quaternionf tipParent = lastDelta.mul(new Quaternionf(tipFrame.parentRotation()));
            Quaternionf tipLocal = tipParent.conjugate().mul(r.tipTarget());
            Quaternionf evaluated = evaluatedRotation(model, tipId);

            if (evaluated == null)
            {
                continue;
            }

            writeOrient(model, tipId, r.weight() >= 1F - EPS ? tipLocal : evaluated.slerp(tipLocal, r.weight()));
        }

        for (ResolvedChain r : resolved)
        {
            if (r.chain().stretch())
            {
                stretchToTarget(model, nodes, tree, r, frames, blendedParentOf, blendedWorld);
            }
        }
    }

    /**
     * Telescopes a chain that came up short onto its controller: whatever gap the
     * rotation solve could not close is split among the chain's bones in
     * proportion to their lengths and written as per-bone translations, so every
     * joint slides out along the limb and the tip lands on the target. No bone is
     * scaled — cubes keep their proportions and their texels, and the joints that
     * open up are sealed by the model's welds.
     *
     * <p>A post-process on purpose: the solve itself stays a pure rotation
     * problem, exactly as it is without stretching, so nothing about a chain's
     * bend, pole or limits changes when the box is ticked — the chain simply
     * stops falling short. The gap is faded by the chain's weight, so stretch
     * comes and goes with the rest of the IK.
     *
     * <p>The share is distributed only up to the last bone carrying GEOMETRY: a
     * chain ending in a bare end-marker (the auto-tail convention) would
     * otherwise open its last seam BEFORE the marker and leave the last visible
     * bone short of the controller.
     */
    private static void stretchToTarget(IModel model, List<String> nodes, IKTree tree, ResolvedChain r, Map<String, PivotFrame> frames, Quaternionf[] blendedParentOf, Quaternionf[] blendedWorld)
    {
        List<String> workIds = r.workIds();
        int lastJoint = indexOf(nodes, workIds.get(workIds.size() - 2));
        int effectorIndex = -1;

        for (int e = 0; e < tree.effectors.length; e++)
        {
            if (tree.effectors[e].joint == lastJoint)
            {
                effectorIndex = e;
                break;
            }
        }

        if (effectorIndex < 0 || lastJoint < 0)
        {
            return;
        }

        /* The effector bone is not a solver node, so its parent frame is not in
         * the blended walk: it is its CAPTURED parent frame carried by how far
         * the last directed bone turned — the same advance the tip snap makes.
         * Using the captured frame raw would send its share of the gap off in
         * the pre-solve direction, which on a chain that turned a long way is a
         * visible miss. */
        Quaternionf tipParent = null;
        PivotFrame tipFrame = frames.get(workIds.get(workIds.size() - 1));

        if (tipFrame != null)
        {
            tipParent = new Quaternionf(blendedWorld[lastJoint])
                .mul(new Quaternionf(tree.joints[lastJoint].startWorldRotation).conjugate())
                .mul(tipFrame.parentRotation());
        }

        Vector3f gap = new Vector3f(r.target()).sub(tree.effectors[effectorIndex].position).mul(r.weight());

        if (gap.lengthSquared() < EPS * EPS)
        {
            return;
        }

        int reach = lastGeometryIndex(model, workIds);

        if (reach < 1)
        {
            return;
        }

        /* Solved positions along the chain: the nodes from the tree, the effector
         * point for the last id. */
        Vector3f[] solved = new Vector3f[workIds.size()];

        for (int i = 0; i < workIds.size(); i++)
        {
            int node = indexOf(nodes, workIds.get(i));

            solved[i] = node >= 0 ? tree.joints[node].position
                : i == workIds.size() - 1 ? tree.effectors[effectorIndex].position : null;

            if (solved[i] == null)
            {
                return;
            }
        }

        float total = 0F;

        for (int i = 0; i < reach; i++)
        {
            total += solved[i].distance(solved[i + 1]);
        }

        if (total < EPS)
        {
            return;
        }

        Vector3f cumulative = new Vector3f();

        for (int i = 1; i <= reach && i < workIds.size(); i++)
        {
            Vector3f share = new Vector3f(gap).mul(solved[i - 1].distance(solved[i]) / total);

            String bone = workIds.get(i);
            int node = indexOf(nodes, bone);
            Quaternionf parentFrame = node >= 0 && blendedParentOf[node] != null ? blendedParentOf[node] : tipParent;

            cumulative.add(share);
            writeStretchOffset(model, bone, frames.get(bone), parentFrame, share, cumulative);
        }
    }

    /**
     * Writes one bone's share of the telescope. Cubic takes the LOCAL step in its
     * parent's frame — the renderer pre-translates there and the matrix stack
     * carries it to the whole subtree, so each bone writes only its own share.
     * Dividing by the frame's scale undoes the scaling the renderer would
     * otherwise apply on top. BOBJ takes the CUMULATIVE world shift into the
     * skinning matrix instead: it is not a hierarchy, and vertices weighted
     * across bones blend neighbouring shifts into a smooth stretch.
     */
    private static void writeStretchOffset(IModel model, String bone, PivotFrame frame, Quaternionf parentFrame, Vector3f share, Vector3f cumulative)
    {
        if (model instanceof BOBJModel bobj)
        {
            BOBJBone bobjBone = bobj.getArmature().bones.get(bone);

            if (bobjBone != null)
            {
                bobjBone.offset = new Vector3f(cumulative);
            }

            return;
        }

        if (model instanceof Model cubic && parentFrame != null)
        {
            ModelGroup group = cubic.getGroup(bone);

            if (group != null)
            {
                Vector3f local = new Quaternionf(parentFrame).conjugate().transform(new Vector3f(share));
                Vector3f scale = frame == null ? null : frame.scale();

                if (scale != null)
                {
                    local.set(divide(local.x, scale.x), divide(local.y, scale.y), divide(local.z, scale.z));
                }

                group.offset = local;
            }
        }
    }

    private static float divide(float value, float by)
    {
        return Math.abs(by) < EPS ? value : value / by;
    }

    private static int indexOf(List<String> nodes, String bone)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            if (nodes.get(i).equals(bone))
            {
                return i;
            }
        }

        return -1;
    }

    /**
     * The deepest chain bone that carries geometry — the bone whose far end
     * should land on the controller when stretching. Trailing bones with no
     * cubes or meshes are bare reach markers (the auto-tail convention); they
     * ride the last real bone's shift instead of opening a seam of their own.
     * Falls back to the last bone when nothing in the chain has geometry.
     */
    private static int lastGeometryIndex(IModel model, List<String> chainIds)
    {
        if (model instanceof Model cubic)
        {
            for (int i = chainIds.size() - 1; i >= 0; i--)
            {
                ModelGroup bone = cubic.getGroup(chainIds.get(i));

                if (bone != null && (!bone.cubes.isEmpty() || !bone.meshes.isEmpty()))
                {
                    return i;
                }
            }
        }

        return chainIds.size() - 1;
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
}
