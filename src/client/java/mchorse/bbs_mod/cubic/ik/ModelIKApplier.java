package mchorse.bbs_mod.cubic.ik;

import mchorse.bbs_mod.bobj.BOBJBone;
import mchorse.bbs_mod.cubic.IModel;
import mchorse.bbs_mod.cubic.constraints.ModelConstraintsConfig.BoneConstraint;
import mchorse.bbs_mod.cubic.data.model.Model;
import mchorse.bbs_mod.cubic.data.model.ModelGroup;
import mchorse.bbs_mod.cubic.ik.solver.IKChain;
import mchorse.bbs_mod.cubic.ik.solver.IKChainSolver;
import mchorse.bbs_mod.cubic.ik.solver.IKJoint;
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
import java.util.List;
import java.util.Map;
import java.util.Set;

final class ModelIKApplier
{
    /**
     * Per-frame solve dump to {@code run/ik-log.txt} (truncated every frame, so the
     * file always holds the LAST frame's chains) — the IK counterpart of the drag
     * log, and like it the thing to ask for when a solve is disputed: captured
     * angles in, goal, iterations, error, angles out. Flip to enable; zero cost off.
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

    public static void apply(IModel model, List<ModelIKCache.CompiledChain> chains, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides, Map<String, BoneConstraint> boneLimits)
    {
        if (model == null || chains == null || chains.isEmpty())
        {
            return;
        }

        /* Apply ancestor chains (shallower root) first, and re-collect frames per
         * chain, so a child chain (e.g. an arm) sees the pose its parent chain
         * (e.g. the body) already produced and rides along with it. */
        List<ModelIKCache.CompiledChain> ordered = new ArrayList<>(chains);
        ordered.sort(Comparator.comparingInt((ModelIKCache.CompiledChain chain) -> rootDepth(model, chain)));

        if (LOG_IK)
        {
            LOG.setLength(0);
        }

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

            applyChain(model, chain, frames, jointDoF, controllerTargets, poleTargets, targetWeights, poleWeights, controlOverrides, boneLimits);
        }

        if (LOG_IK)
        {
            flushLog();
        }
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

    private static void applyChain(IModel model, ModelIKCache.CompiledChain chain, Map<String, PivotFrame> frames, Map<String, ModelIKConfig.JointDoF> jointDoF, Map<String, Vector3f> controllerTargets, Map<String, Vector3f> poleTargets, Map<String, Float> targetWeights, Map<String, Float> poleWeights, Map<String, IKControl> controlOverrides, Map<String, BoneConstraint> boneLimits)
    {
        /* The film's `ik` track may override the chain's static config scalars.
         * IK weight is independent of pose `fix` — freezing a bone pins it to rest
         * (changing the FK pose IK reads from) but no longer gates IK weight, which
         * comes only from the config and the `ik` track. */
        IKControl control = controlOverrides == null ? null : controlOverrides.get(chain.tip());

        if (control != null && !control.enabled)
        {
            return;
        }

        boolean pole = control != null ? control.pole : chain.pole();
        float softness = control != null ? control.softness : chain.softness();
        float weight = control != null ? control.weight : chain.weight();

        float poleAngle = (float) Math.toRadians(control != null ? control.poleAngle : chain.poleAngle());

        if (weight <= 0F)
        {
            return;
        }

        PivotFrame targetFrame = frames.get(chain.target());

        if (targetFrame == null)
        {
            return;
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

        if (model instanceof Model cubic)
        {
            applyChainCubic(cubic, workIds, frames, jointDoF, target, pole, polePoint, poleAngle, softness, weight, tipTarget);
        }
        else if (model instanceof BOBJModel bobj)
        {
            applyChainBobjLegacy(bobj, workIds, frames, target, pole, polePoint, poleAngle, softness, weight, tipTarget, boneLimits);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Cubic: channel-space damped-least-squares solve                     */
    /* ------------------------------------------------------------------ */

    /**
     * The cubic solve: capture the chain from the pivot frames into the
     * channel-space core ({@link IKChain}), seed the bend on a dead-straight
     * limb from its authored rest bend, soften the goal, run the DLS solve
     * (with Blender's pole), and write each bone's solved local rotation to
     * {@link ModelGroup#orient} blended against the FK base by the IK weight.
     * The euler channels are never touched — they stay the read-only FK truth
     * (the constraint-stack contract), and the solved angles START from them,
     * so the twist the animator posed survives into the solve by construction.
     */
    private static void applyChainCubic(Model model, List<String> workIds, Map<String, PivotFrame> frames, Map<String, ModelIKConfig.JointDoF> jointDoF, Vector3f target, boolean pole, Vector3f polePoint, float poleAngle, float softness, float weight, Quaternionf tipTarget)
    {
        IKChain chain = captureChain(model, workIds, frames, jointDoF);

        if (chain == null)
        {
            return;
        }

        /* Pole ON with no pole target bone = the REST-AUTHORED virtual pole: the bend
         * plane is stabilized towards the side the model was built bent towards (knee
         * forward, elbow back), lifted by the chain root's current parent frame so it
         * tracks the shoulder/hip. Without it the bend side is ambiguous and a target
         * orbiting the character makes the chain SNAP sides at the degenerate
         * directions (Blender behaves the same — its riggers always add a pole; this
         * bakes that rule in). A rig with no authored bend (a rope) has no side to
         * offer and keeps the pure pose-driven behavior. */
        if (pole && polePoint == null)
        {
            polePoint = restVirtualPole(model, workIds, chain);
        }

        Vector3f goal = IKChainSolver.softGoal(chain, target, softness);

        if (LOG_IK)
        {
            logChainIn(workIds, chain, target, goal, polePoint, weight);
        }

        IKChainSolver.Result result = IKChainSolver.solve(chain, goal, polePoint, poleAngle, IKChainSolver.Params.DEFAULT);

        if (LOG_IK)
        {
            logChainOut(chain, result);
        }

        writeOrientations(model, workIds, chain, weight, tipTarget);
    }

    private static void logChainIn(List<String> workIds, IKChain chain, Vector3f target, Vector3f goal, Vector3f polePoint, float weight)
    {
        LOG.append("chain ").append(workIds).append(" weight ").append(weight).append('\n');
        LOG.append("  target ").append(fmt(target)).append(" goal ").append(fmt(goal));
        LOG.append(" pole ").append(polePoint == null ? "-" : fmt(polePoint)).append('\n');

        for (int i = 0; i < chain.joints.length; i++)
        {
            LOG.append("  in  ").append(workIds.get(i)).append(" angles ").append(fmtDeg(chain.joints[i].angles))
                .append(" pos ").append(fmt(chain.joints[i].startPosition)).append('\n');
        }
    }

    private static void logChainOut(IKChain chain, IKChainSolver.Result result)
    {
        LOG.append("  solved reached=").append(result.reached()).append(" err=").append(result.error())
            .append(" iters=").append(result.iterations()).append(" effector ").append(fmt(chain.effector)).append('\n');

        for (IKJoint joint : chain.joints)
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
     * Captures the chain's directed bones (all but the last work id) into solver
     * joints: world frames straight from the pivot frames, start angles from the
     * bone's evaluated rotation — the rotation the renderer actually applies —
     * decomposed continuously against the euler channels, so a quaternion-mode
     * bone or a composed layer stack enters the solve without a branch flip and
     * a plain euler bone enters byte-exact. Each joint picks up its per-bone
     * freedom from the config's "bones" section (locks, degree limits converted
     * to the solver's radians, stiffness). Returns {@code null} when a frame or
     * bone is missing (the solve is silently skipped, as before).
     */
    private static IKChain captureChain(Model model, List<String> workIds, Map<String, PivotFrame> frames, Map<String, ModelIKConfig.JointDoF> jointDoF)
    {
        int directed = workIds.size() - 1;

        if (directed < 1)
        {
            return null;
        }

        PivotFrame rootFrame = frames.get(workIds.get(0));
        PivotFrame effectorFrame = frames.get(workIds.get(workIds.size() - 1));

        if (rootFrame == null || effectorFrame == null)
        {
            return null;
        }

        IKChain chain = new IKChain(directed);

        chain.rootParentRotation.set(rootFrame.parentRotation());
        chain.startEffector.set(effectorFrame.position());

        for (int i = 0; i < directed; i++)
        {
            ModelGroup bone = model.getGroup(workIds.get(i));
            PivotFrame frame = frames.get(workIds.get(i));

            if (bone == null || frame == null)
            {
                return null;
            }

            IKJoint joint = chain.joints[i];

            joint.startPosition.set(frame.position());
            joint.startWorldRotation.set(frame.worldRotation());
            sourceAngles(bone, joint.startAngles);
            joint.angles.set(joint.startAngles);

            ModelIKConfig.JointDoF dof = jointDoF == null ? null : jointDoF.get(workIds.get(i));

            if (dof != null)
            {
                applyDoF(joint, dof);
            }
        }

        return chain;
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

    /**
     * The rest-authored virtual pole point: the direction the chain's first
     * interior pivot sticks out from the rest root-to-effector line — where the
     * model's own elbow/knee points — lifted into the world by the chain root's
     * current parent frame (the same lift {@link #restBendNormal} uses) and
     * placed a chain-length away from the root. {@code null} when the chain is
     * too short or authored dead straight (no side to prefer).
     */
    private static Vector3f restVirtualPole(Model model, List<String> workIds, IKChain chain)
    {
        if (chain.joints.length < 2)
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

        chain.rootParentRotation.transform(side);

        return new Vector3f(chain.joints[0].startPosition).fma(chain.totalLength(), side);
    }

    /**
     * Writes the solved chain onto the bones: each directed bone's local rotation
     * is composed from its solved channel angles and written raw to
     * {@link ModelGroup#orient}, blended against the FK base (the bone's
     * evaluated rotation) by the IK weight. The parent world frame advances by
     * the APPLIED (blended) rotation — the frame the renderer establishes — so
     * the tip's target orientation lands in the right frame at any weight.
     */
    private static void writeOrientations(Model model, List<String> workIds, IKChain chain, float weight, Quaternionf tipTarget)
    {
        Quaternionf parentWorld = new Quaternionf(chain.rootParentRotation);

        for (int i = 0; i < chain.joints.length; i++)
        {
            ModelGroup bone = model.getGroup(workIds.get(i));

            if (bone == null)
            {
                return;
            }

            Quaternionf solvedLocal = Matrices.toLocalRotationZYXRadians(chain.joints[i].angles);
            Quaternionf applied = weight >= 1F - EPS ? solvedLocal : bone.evaluatedRotation().slerp(solvedLocal, weight);

            bone.orient = applied;
            parentWorld.mul(applied);
        }

        /* Tip follows target: the effector (last id, not a solver joint) copies the
         * controller's world orientation. parentWorld is now the tip's parent frame. */
        if (tipTarget != null)
        {
            ModelGroup tip = model.getGroup(workIds.get(workIds.size() - 1));

            if (tip == null)
            {
                return;
            }

            Quaternionf tipLocal = new Quaternionf(parentWorld).conjugate().mul(tipTarget);

            tip.orient = weight >= 1F - EPS ? tipLocal : tip.evaluatedRotation().slerp(tipLocal, weight);
        }
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

    private static void applyChainBobjLegacy(BOBJModel model, List<String> workIds, Map<String, PivotFrame> frames, Vector3f target, boolean pole, Vector3f polePoint, float poleAngle, float softness, float weight, Quaternionf tipTarget, Map<String, BoneConstraint> boneLimits)
    {
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
        List<Vector3f> solved = IKSolver.solve(currentPositions, target, pole, polePoint, poleAngle, softness, LEGACY_MAX_ITERATIONS, LEGACY_TOLERANCE, limits, limits == null ? null : rootParentRotation, restHinge, bendNormal);

        /* The bend-plane normal the solve settled on (zero when undefined). Passed to the
         * orientation pass as the roll-reference seed so a fully straight chain keeps a
         * stable twist instead of jittering at the reach boundary. */
        Vector3f bendSeed = bendNormal.lengthSquared() < EPS * EPS ? null : bendNormal;

        if (workIds.size() >= 3)
        {
            buildChainOrientationsBobj(model, workIds, solved, rootParentRotation, weight, tipTarget, bendSeed);
        }
        else
        {
            Vector3f[] solvedArray = solved.toArray(new Vector3f[solved.size()]);
            ModelRotationBlender.applyWeightedRotations(model, rootParentRotation, workIds, solvedArray, weight);
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
     * constraints config (the cubic path's limits move to the IK config's
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
