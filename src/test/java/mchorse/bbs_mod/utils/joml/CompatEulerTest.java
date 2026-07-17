package mchorse.bbs_mod.utils.joml;

import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Guards the mod's own euler readbacks ({@link Matrices#toCompatibleEulerZYXRadians},
 * {@link Matrices#toEulerZYXRadians}) — THE canonical matrix/quat&rarr;euler decompositions — against
 * the pole/branch/winding regressions that eye-testing could not catch, and against the next silent
 * JOML swap by Minecraft: the JOML bundled with MC 1.20.4 (1.10.5, pinned in the test classpath)
 * decomposes WRONG (its quat path returns angles up to 180° off past the pole, its matrix path NaNs
 * at exactly ±90°), which is why the mod owns these numerics. The reference paths come from a real
 * drag log ({@code run/drag-log.txt}) of the trackball bug this math fixed.
 *
 * <p>The anchor taxonomy under test: a readback has TWO anchors and they answer independent
 * questions. The BRANCH (which of the two euler families) strands where a path passes beside the
 * pole, so it anchors at the GRAB and the write stays a pure function of the gesture. The WINDING
 * (whole turns on top) can only be known from the previous value, so it always anchors LIVE and a
 * spin keeps counting past ±180°. They shared one reference until 2026-07-17, which looked like a
 * forced trade between the two — {@link #trackballWindsPastHalfTurn} is the bug that exposed it.
 */
public class CompatEulerTest
{
    /** The exact per-frame orientations from drag-log.txt (trackball, wobbly Y-sweep through the pole). */
    @Test
    public void logPathGrabAnchored()
    {
        float[][] quats = {
            {+0.9986f, -0.0044f, +0.0522f, -0.0043f}, // #2   ~6°
            {+0.9936f, -0.0043f, +0.1128f, -0.0094f}, // #3   ~13°
            {+0.9863f, +0.0000f, +0.1644f, -0.0144f}, // #4   ~19°
            {+0.9763f, +0.0043f, +0.2155f, -0.0198f}, // #5   ~25°
            {+0.9648f, +0.0084f, +0.2618f, -0.0252f}, // #6   ~30.5°
            {+0.9497f, +0.0083f, +0.3117f, -0.0300f}, // #7   ~36.5°
            {+0.9335f, +0.0081f, +0.3567f, -0.0343f}, // #8   ~42°
            {+0.9170f, +0.0120f, +0.3967f, -0.0399f}, // #9   ~47°
            {+0.8968f, +0.0117f, +0.4401f, -0.0443f}, // #10  ~52.5°
            {+0.8725f, +0.0076f, +0.4864f, -0.0468f}, // #11  ~58.5°
            {+0.8457f, +0.0037f, +0.5314f, -0.0488f}, // #12  ~64.5°
            {+0.8191f, -0.0036f, +0.5716f, -0.0475f}, // #13  ~70°
            {+0.7906f, -0.0103f, +0.6105f, -0.0454f}, // #14  ~75.5°
            {+0.7601f, -0.0232f, +0.6484f, -0.0368f}, // #15  ~81°
            {+0.7307f, -0.0319f, +0.6813f, -0.0297f}, // #16  ~86°  (pole approach)
            {+0.7004f, -0.0275f, +0.7124f, -0.0342f}, // #17  ~91°  (pole)
            {+0.6723f, -0.0117f, +0.7384f, -0.0516f}, // #18  ~95.5°
            {+0.6361f, +0.0000f, +0.7687f, -0.0673f}, // #19  ~101°
            {+0.6018f, +0.0053f, +0.7950f, -0.0765f}, // #20  ~106°
            {+0.5555f, +0.0097f, +0.8269f, -0.0869f}, // #21  ~112.5°
            {+0.5112f, +0.0112f, +0.8543f, -0.0936f}, // #22  ~118.5°
            {+0.4693f, +0.0123f, +0.8773f, -0.1000f}, // #23  ~124°
            {+0.4065f, +0.0124f, +0.9072f, -0.1074f}, // #24  ~132°
            {+0.3622f, +0.0126f, +0.9251f, -0.1136f}, // #25  ~137.5°
            {+0.3046f, +0.0120f, +0.9448f, -0.1202f}, // #26  ~144.5°
            {+0.2586f, +0.0102f, +0.9582f, -0.1219f}, // #27  ~150°
        };

        Vector3f grabRef = new Vector3f(0F, 0F, 0F);
        Vector3f euler = new Vector3f();
        float prevY = -999F;
        boolean yMonotonic = true;
        boolean reproduced = true;

        for (float[] wxyz : quats)
        {
            Quaternionf q = new Quaternionf(wxyz[1], wxyz[2], wxyz[3], wxyz[0]); // JOML ctor: x,y,z,w

            Matrices.toCompatibleEulerZYXRadians(q, grabRef, euler);

            float yd = deg(euler.y);

            if (prevY != -999F && yd < prevY - 0.5F) yMonotonic = false;
            prevY = yd;

            if (reproErr(euler, q) > 0.05F) reproduced = false;
        }

        assertTrue(reproduced, "every written value reproduces the exact orientation");
        assertTrue(yMonotonic, "Y grows monotonically through the pole (no fold-back onto the flipped branch)");
        assertTrue(Math.abs(deg(euler.x)) < 20F && Math.abs(deg(euler.z)) < 20F,
            String.format("final X/Z small after the pole transient decays: (%.1f, %.1f)", deg(euler.x), deg(euler.z)));
        assertTrue(Math.abs(deg(euler.y) - 150F) < 3F, String.format("final Y ~150 (%.1f)", deg(euler.y)));
    }

    /** A perfectly clean Y-sweep (no wobble): grab-anchor must give exactly (0, θ, 0). */
    @Test
    public void cleanCorridorGrabAnchored()
    {
        Vector3f grabRef = new Vector3f();
        Vector3f euler = new Vector3f();

        for (int t = 5; t <= 175; t += 5)
        {
            Quaternionf q = new Quaternionf().rotationY(rad(t));

            Matrices.toCompatibleEulerZYXRadians(q, grabRef, euler);

            assertTrue(Math.abs(deg(euler.x)) < 0.01F && Math.abs(deg(euler.y) - t) < 0.01F && Math.abs(deg(euler.z)) < 0.01F,
                String.format("clean corridor at θ=%d stays (0, θ, 0): got (%.2f, %.2f, %.2f)", t, deg(euler.x), deg(euler.y), deg(euler.z)));
        }
    }

    /**
     * Ring regression, single-anchor flavour: a LIVE-anchored corridor must wind through
     * 90/180/270/360… up to 1080° with no NaN at the exact poles (JOML 1.10.5's matrix path NaNs at
     * ±90.00° — the ring snaps onto 90.00 and used to write NaN into bone channels). The ring now
     * goes through the split-anchor call ({@link #ringCorridorWindsWithGrabBranch}); this keeps
     * guarding the one-reference primitive, which the IK blends and the migrations still use.
     */
    @Test
    public void ringCorridorWindsLiveAnchored()
    {
        Vector3f ref = new Vector3f(); // walks with each write, like the ring's live channels
        Vector3f euler = new Vector3f();

        for (int t = 5; t <= 1080; t += 5)
        {
            Quaternionf q = new Quaternionf().rotationY(rad(t % 360));

            Matrices.toCompatibleEulerZYXRadians(q, ref, euler);
            ref.set(euler);

            boolean finite = !Float.isNaN(euler.x) && !Float.isNaN(euler.y) && !Float.isNaN(euler.z);

            assertTrue(finite && Math.abs(deg(euler.y) - t) < 0.02F && Math.abs(deg(euler.x)) < 0.01F && Math.abs(deg(euler.z)) < 0.01F,
                String.format("live-anchored corridor at θ=%d winds exactly, no NaN: got (%.2f, %.2f, %.2f)", t, deg(euler.x), deg(euler.y), deg(euler.z)));
        }
    }

    /** A pre-wound pose (y=720) + small further turn, grab-anchored: the winding must survive the anchor. */
    @Test
    public void woundGrabAnchorKeepsWinding()
    {
        Vector3f grabRef = new Vector3f(0F, rad(720F), 0F);
        Quaternionf q = new Quaternionf().rotationY(rad(10F)); // 730 ≡ 10
        Vector3f euler = new Vector3f();

        Matrices.toCompatibleEulerZYXRadians(q, grabRef, euler);

        assertTrue(Math.abs(deg(euler.y) - 730F) < 0.02F && Math.abs(deg(euler.x)) < 0.01F,
            String.format("(0,720,0)+10° -> (%.2f, %.2f, %.2f), expect y=730", deg(euler.x), deg(euler.y), deg(euler.z)));
    }

    /** Exact-pole orientations: JOML's matrix path NaNs there, JOML's quat path loses the twist. */
    @Test
    public void exactPoleNoNaNTwistPreserved()
    {
        Vector3f euler = new Vector3f();

        Quaternionf pure = new Quaternionf().rotationY(rad(90F));
        Matrices.toCompatibleEulerZYXRadians(pure, new Vector3f(0F, rad(85F), 0F), euler);
        assertTrue(!Float.isNaN(euler.y) && Math.abs(deg(euler.y) - 90F) < 0.01F && Math.abs(deg(euler.x)) < 0.01F,
            String.format("Ry(90) exact, ref (0,85,0) -> (%.2f, %.2f, %.2f)", deg(euler.x), deg(euler.y), deg(euler.z)));

        Quaternionf twisted = new Quaternionf().rotationY(rad(90F)).rotateX(rad(30F));
        Matrices.toCompatibleEulerZYXRadians(twisted, new Vector3f(rad(25F), rad(85F), 0F), euler);
        assertTrue(reproErr(euler, twisted) < 0.01F,
            String.format("Ry(90)·Rx(30) reproduces exactly -> (%.2f, %.2f, %.2f)", deg(euler.x), deg(euler.y), deg(euler.z)));

        /* The gun default transform: middle angle exactly -90 (south pole) — was CORRUPTED by
         * JOML's broken decomposition between the rotate2 migration and the eulerZYXRaw fix. */
        Quaternionf gun = new Quaternionf().rotationZYX(0F, -rad(90F), 0F).mul(new Quaternionf().rotationZYX(rad(45F), 0F, 0F));
        Vector3f gunEuler = Matrices.toEulerZYXRadians(gun, new Vector3f());
        assertTrue(reproErr(gunEuler, gun) < 0.01F,
            String.format("gun Ry(-90)·Rz(45) principal reproduces -> (%.2f, %.2f, %.2f)", deg(gunEuler.x), deg(gunEuler.y), deg(gunEuler.z)));
    }

    /** The safe principal readback vs JOML 1.10.5's broken quat decomposition on beyond-pole poses. */
    @Test
    public void safePrincipalReadback()
    {
        Vector3f euler = new Vector3f();

        Quaternionf ry150 = new Quaternionf().rotationY(rad(150F));
        Matrices.toEulerZYXRadians(ry150, euler);
        assertTrue(reproErr(euler, ry150) < 0.01F,
            String.format("Ry(150) -> (%.2f, %.2f, %.2f) reproduces (JOML 1.10.5 quat gave (0,30,180) = 180° off)",
                deg(euler.x), deg(euler.y), deg(euler.z)));

        Quaternionf log17 = new Quaternionf(-0.0275f, 0.7124f, -0.0342f, 0.7004f);
        Matrices.toEulerZYXRadians(log17, euler);
        assertTrue(reproErr(euler, log17) < 0.05F, "log #17 quat reproduces (JOML 1.10.5 was 98° off)");
    }

    /* ── The split anchors (branch at the grab, winding live) ─────────────────────────────── */

    /**
     * Passing one reference for both anchors must stay EXACTLY the pre-split algorithm — every
     * non-drag caller (IK blending, constraints, the mode toggle, the rotate2 migration) goes
     * through that flavour and must not shift by a bit. Fuzzed, with half the samples parked on the
     * pole where the branch pick actually bites.
     */
    @Test
    public void singleAnchorIsUnchangedByTheSplit()
    {
        Random rng = new Random(20260717L);
        float worst = 0F;

        for (int i = 0; i < 50000; i++)
        {
            Vector3f angles = randomEuler(rng);
            Vector3f ref = randomEuler(rng);

            if ((i & 1) == 0) angles.y = rad(90F) + (rng.nextFloat() - 0.5F) * 0.02F;

            Matrix3f m = zyx(angles);
            Vector3f split = Matrices.toCompatibleEulerZYXRadians(m, ref, ref, new Vector3f());
            Vector3f single = Matrices.toCompatibleEulerZYXRadians(m, ref, new Vector3f());

            worst = Math.max(worst, split.distance(single));
        }

        assertTrue(worst == 0F, "same reference twice == the one-reference flavour, bit for bit (max deviation " + worst + ")");
    }

    /**
     * The trackball bug Вемпи caught on 2026-07-17 (fixed in c73a4d8f0). Spinning past half a turn
     * kept turning the bone, but the channel folded — the log read ry +178.00 → −175.01 → −168.52
     * while the quaternion advanced smoothly and X/Z never moved, i.e. the euler FAMILY was fine and
     * only the winding folded. Cause: one anchor doing both jobs — the grab caps the value within
     * half a turn of itself by construction. The winding must count from the live channels.
     */
    @Test
    public void trackballWindsPastHalfTurn()
    {
        Vector3f grab = new Vector3f(rad(-9.06F), 0F, rad(-0.11F));
        Vector3f live = new Vector3f(grab);
        float previous = Float.NEGATIVE_INFINITY;

        /* half a degree per frame, out to a turn and a half */
        for (int step = 1; step <= 1080; step++)
        {
            Matrix3f target = new Matrix3f().rotation(rad(step * 0.5F), 0F, 1F, 0F).mul(zyx(grab));
            Vector3f euler = Matrices.toCompatibleEulerZYXRadians(target, grab, live, new Vector3f());

            assertTrue(euler.y >= previous - 1.0E-4F,
                String.format("Y never folds back: %.2f after %.2f at sweep %.1f°", deg(euler.y), deg(previous), step * 0.5F));
            assertTrue(reproErr(euler, target) < 0.05F, "every written value reproduces the exact orientation");

            previous = euler.y;
            live.set(euler);
        }

        assertTrue(Math.abs(deg(previous) - 540F) < 1F, String.format("a 540° spin reads 540, not folded (%.1f)", deg(previous)));

        /* The same input through the old one-anchor call still folds — this is the regression itself. */
        Matrix3f at185 = new Matrix3f().rotation(rad(185F), 0F, 1F, 0F).mul(zyx(grab));
        float folded = deg(Matrices.toCompatibleEulerZYXRadians(at185, grab, new Vector3f()).y);
        float wound = deg(Matrices.toCompatibleEulerZYXRadians(at185, grab, new Vector3f(grab.x, rad(178F), grab.z), new Vector3f()).y);

        assertTrue(folded < -170F && Math.abs(wound - 185F) < 1F,
            String.format("grab-wound folds (%.2f, the logged −175.01), live-wound doesn't (%.2f)", folded, wound));
    }

    /**
     * The ring's own regression, now that it goes through the split call: a grab-anchored BRANCH
     * must not cost the >360° winding the ring is built for. Corridor to 1080° through every pole,
     * exact, no NaN.
     */
    @Test
    public void ringCorridorWindsWithGrabBranch()
    {
        Vector3f grab = new Vector3f();
        Vector3f live = new Vector3f();

        for (int t = 5; t <= 1080; t += 5)
        {
            Matrix3f target = new Matrix3f().rotation(rad(t), 0F, 1F, 0F);
            Vector3f euler = Matrices.toCompatibleEulerZYXRadians(target, grab, live, new Vector3f());

            live.set(euler);

            boolean finite = !Float.isNaN(euler.x) && !Float.isNaN(euler.y) && !Float.isNaN(euler.z);

            assertTrue(finite && Math.abs(deg(euler.y) - t) < 0.02F && Math.abs(deg(euler.x)) < 0.01F && Math.abs(deg(euler.z)) < 0.01F,
                String.format("grab-branch corridor at θ=%d winds exactly, no NaN: got (%.2f, %.2f, %.2f)", t, deg(euler.x), deg(euler.y), deg(euler.z)));
        }
    }

    /**
     * Why the ring took the grab branch too (2026-07-17): a sweep whose bone sits slightly off the
     * corridor passes BESIDE the pole — a branch point of the euler map — and a live-anchored branch
     * flows onto the flipped family and latches there, parking X/Z near ±180 for the rest of the
     * gesture though the orientation only turned about one axis. The grab anchor re-picks the family
     * against a fixed pose, so it comes back. Both are the same orientation; only one is a sane
     * channel value to keyframe.
     */
    @Test
    public void nearPoleStrandRecoversWithGrabBranch()
    {
        Vector3f grab = new Vector3f(rad(12F), 0F, rad(6F));
        Vector3f liveBranch = new Vector3f(grab);
        Vector3f grabBranch = new Vector3f(grab);
        Vector3f strandedOuter = new Vector3f();
        Vector3f recoveredOuter = new Vector3f();

        for (int t = 1; t <= 180; t++)
        {
            Matrix3f target = new Matrix3f().rotation(rad(t), 0F, 1F, 0F).mul(zyx(grab));

            /* live branch: the anchor walks with the value (the ring's old behaviour) */
            Vector3f a = Matrices.toCompatibleEulerZYXRadians(target, liveBranch, liveBranch, new Vector3f());
            liveBranch.set(a);

            /* grab branch: family re-picked against the fixed grab, winding still live */
            Vector3f b = Matrices.toCompatibleEulerZYXRadians(target, grab, grabBranch, new Vector3f());
            grabBranch.set(b);

            assertTrue(reproErr(b, target) < 0.05F, "the grab-branch write reproduces the orientation at every step");

            if (t == 180)
            {
                strandedOuter.set(Math.abs(deg(a.x)), 0F, Math.abs(deg(a.z)));
                recoveredOuter.set(Math.abs(deg(b.x)), 0F, Math.abs(deg(b.z)));
            }
        }

        assertTrue(Math.abs(recoveredOuter.x - 12F) < 1F && Math.abs(recoveredOuter.z - 6F) < 1F,
            String.format("grab branch returns the outer angles to the grab's own (%.1f, %.1f), expect (12, 6)", recoveredOuter.x, recoveredOuter.z));
        assertTrue(strandedOuter.x > 90F || strandedOuter.z > 90F,
            String.format("the live branch is what strands them near ±180 (%.1f, %.1f) — if this ever fails, the ring's grab branch is no longer buying anything", strandedOuter.x, strandedOuter.z));
    }

    private static Vector3f randomEuler(Random rng)
    {
        return new Vector3f(
            (rng.nextFloat() - 0.5F) * 4F * (float) Math.PI,
            (rng.nextFloat() - 0.5F) * 4F * (float) Math.PI,
            (rng.nextFloat() - 0.5F) * 4F * (float) Math.PI
        );
    }

    /** {@code Rz·Ry·Rx} — the renderer's composition order, same as RotationDragMath.eulerZYX. */
    private static Matrix3f zyx(Vector3f radians)
    {
        return new Matrix3f().rotationZ(radians.z).rotateY(radians.y).rotateX(radians.x);
    }

    private static float deg(float rad)
    {
        return (float) Math.toDegrees(rad);
    }

    private static float rad(float deg)
    {
        return (float) Math.toRadians(deg);
    }

    /** See {@link #reproErr(Vector3f, Quaternionf)}; against a rotation matrix. */
    private static float reproErr(Vector3f eulerRad, Matrix3f target)
    {
        return reproErr(eulerRad, new Quaternionf().setFromNormalized(target));
    }

    /** Degrees of orientation error when re-composing the euler triple ZYX. */
    private static float reproErr(Vector3f eulerRad, Quaternionf target)
    {
        Quaternionf back = new Quaternionf().rotationZYX(eulerRad.z, eulerRad.y, eulerRad.x);
        Quaternionf d = back.conjugate().mul(new Quaternionf(target).normalize()).normalize();

        return (float) Math.toDegrees(2.0 * Math.acos(Math.min(1F, Math.abs(d.w))));
    }
}
