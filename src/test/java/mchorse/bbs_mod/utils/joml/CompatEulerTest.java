package mchorse.bbs_mod.utils.joml;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

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
 * <p>The reference-anchor taxonomy under test: GRAB-anchored readback (free rotations — trackball,
 * arcball, view) is a pure function of the gesture and self-recovers past the pole; LIVE-anchored
 * readback (the ring) keeps strict continuity and winds beyond 360°. Every decomposition site picks
 * one consciously.
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
     * Ring regression: a LIVE-anchored corridor must wind through 90/180/270/360… up to 1080° with
     * no NaN at the exact poles (JOML 1.10.5's matrix path NaNs at ±90.00° — the ring snaps onto
     * 90.00 and used to write NaN into bone channels).
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

    private static float deg(float rad)
    {
        return (float) Math.toDegrees(rad);
    }

    private static float rad(float deg)
    {
        return (float) Math.toRadians(deg);
    }

    /** Degrees of orientation error when re-composing the euler triple ZYX. */
    private static float reproErr(Vector3f eulerRad, Quaternionf target)
    {
        Quaternionf back = new Quaternionf().rotationZYX(eulerRad.z, eulerRad.y, eulerRad.x);
        Quaternionf d = back.conjugate().mul(new Quaternionf(target).normalize()).normalize();

        return (float) Math.toDegrees(2.0 * Math.acos(Math.min(1F, Math.abs(d.w))));
    }
}
