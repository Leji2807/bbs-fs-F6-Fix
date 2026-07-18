package mchorse.bbs_mod.cubic.ik.solver;

import org.joml.Quaternionf;
import org.joml.Vector3f;

/**
 * One directed bone of an IK chain, as the solver sees it: its channel angles
 * and its stretch — the solver's variables — plus per-axis degrees of freedom,
 * and its captured and working world frames.
 *
 * <p>The solver's primary variable is {@link #angles}: the bone's ZYX rotation
 * channels in radians, the SAME parametrization the renderer composes ({@code
 * Rz·Ry·Rx}), so limits clamp the very numbers the animator sees on the
 * rotation pads and the result folds back into the pose without any
 * reconstruction. A quaternion-mode bone enters through its compatible-euler
 * decomposition and leaves as a quaternion again — the parametrization is
 * internal to the solve.
 *
 * <p>{@link #stretch} is the fourth variable: the RELATIVE elongation of the
 * bone's outgoing segments (0 = its natural length), a translational degree of
 * freedom that lets a chain reach past its rest length. It is dimensionless on
 * purpose — its Jacobian column is the segment vector itself, which carries
 * the length, so a stretch delta and an angle delta move the effector by
 * comparable amounts and share the solver's one step cap.
 *
 * <p>Axis indices are {@code 0 = X, 1 = Y, 2 = Z} throughout; the solver
 * addresses all four variables through {@link #variable(int)} with
 * {@link #STRETCH} as the fourth.
 */
public final class IKJoint
{
    /** Index of the stretch variable, past the three channel axes. */
    public static final int STRETCH = 3;

    /** How many solver variables one joint carries: three channel angles plus stretch. */
    public static final int VARIABLES = 4;

    /* --- captured at build (the FK pose the solve starts from) --- */

    /** World pivot position at capture. */
    public final Vector3f startPosition = new Vector3f();

    /** World rotation at capture (pivot frame AFTER the bone's own rotation). */
    public final Quaternionf startWorldRotation = new Quaternionf();

    /** FK channel angles at capture, ZYX radians — the blend base and the value locked axes hold. */
    public final Vector3f startAngles = new Vector3f();

    /* --- the solver variables --- */

    /** Current channel angles, ZYX radians. Starts equal to {@link #startAngles}. */
    public final Vector3f angles = new Vector3f();

    /**
     * Current relative elongation of this bone's outgoing segments: {@code 0} =
     * the captured (natural) length, {@code 0.5} = half again as long. Always
     * starts at 0 — the capture IS the natural pose, so nothing carries over
     * between frames and the solve stays stateless.
     */
    public float stretch;

    /* --- per-axis degrees of freedom (Blender's bone IK panel) --- */

    /** A locked axis is absent from the Jacobian and FROZEN at its FK value. */
    public final boolean[] locked = new boolean[3];

    /** Whether {@link #limitMin}/{@link #limitMax} apply on this axis. */
    public final boolean[] limited = new boolean[3];

    /** Lower rotation limit per axis, radians. */
    public final float[] limitMin = new float[3];

    /** Upper rotation limit per axis, radians. */
    public final float[] limitMax = new float[3];

    /** 0 = moves freely, approaching 1 = increasingly reluctant to move. */
    public final float[] stiffness = new float[3];

    /**
     * How willing this bone is to stretch, 0..1 — the stretch analogue of
     * {@code 1 - stiffness}. {@code 0} keeps the stretch variable out of the
     * Jacobian entirely, so the bone holds its natural length.
     */
    public float stretchWeight;

    /** Largest relative elongation this bone may take; {@code 0} disables stretching. */
    public float stretchMax;

    /* --- working state, refreshed by IKChain.forward() --- */

    /** World pivot position at the current angles. */
    public final Vector3f position = new Vector3f();

    /** World rotation of the parent frame (pivot frame BEFORE this bone's rotation). */
    public final Quaternionf parentRotation = new Quaternionf();

    /** World rotation at the current angles. */
    public final Quaternionf worldRotation = new Quaternionf();

    /** How willing this axis is to move: {@code 1 - stiffness}, clamped to [0, 1]. */
    public float weight(int axis)
    {
        float w = 1F - this.stiffness[axis];

        return w < 0F ? 0F : Math.min(w, 1F);
    }

    /** The largest factor this bone's outgoing segments can grow by; {@code 1} when it cannot stretch. */
    public float maxStretchScale()
    {
        return this.stretchWeight > 0F && this.stretchMax > 0F ? 1F + this.stretchMax : 1F;
    }

    /** Reads solver variable {@code v}: {@code 0..2} = the ZYX channel angles, {@link #STRETCH} = stretch. */
    public float variable(int v)
    {
        return v == STRETCH ? this.stretch : get(this.angles, v);
    }

    /** Writes solver variable {@code v}; see {@link #variable(int)}. */
    public void variable(int v, float value)
    {
        if (v == STRETCH)
        {
            this.stretch = value;
        }
        else
        {
            set(this.angles, v, value);
        }
    }

    /** Clamps {@link #angles} into the enabled per-axis limits and {@link #stretch} into {@code [0, stretchMax]}. */
    public void clampLimits()
    {
        for (int axis = 0; axis < 3; axis++)
        {
            if (!this.limited[axis])
            {
                continue;
            }

            float value = get(this.angles, axis);
            float clamped = Math.max(this.limitMin[axis], Math.min(this.limitMax[axis], value));

            if (clamped != value)
            {
                set(this.angles, axis, clamped);
            }
        }

        /* Stretch only ever lengthens: a chain that could SHORTEN its bones would
         * cheat its way to a close goal by collapsing, which reads as the limb
         * shrinking rather than reaching. */
        if (this.stretch < 0F)
        {
            this.stretch = 0F;
        }
        else if (this.stretch > this.stretchMax)
        {
            this.stretch = this.stretchMax;
        }
    }

    static float get(Vector3f v, int axis)
    {
        return axis == 0 ? v.x : axis == 1 ? v.y : v.z;
    }

    static void set(Vector3f v, int axis, float value)
    {
        if (axis == 0) v.x = value;
        else if (axis == 1) v.y = value;
        else v.z = value;
    }
}
