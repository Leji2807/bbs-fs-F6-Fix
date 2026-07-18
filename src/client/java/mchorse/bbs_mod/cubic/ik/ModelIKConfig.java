package mchorse.bbs_mod.cubic.ik;

import java.util.Collections;
import java.util.List;
import java.util.Map;

public record ModelIKConfig(List<Chain> chains, Map<String, JointDoF> bones)
{
    public ModelIKConfig
    {
        bones = bones == null ? Collections.emptyMap() : bones;
    }

    public static final float DEFAULT_WEIGHT = 1F;
    public static final String DEFAULT_POLE_TARGET = "";
    public static final float DEFAULT_POLE_ANGLE = 0F;
    public static final float DEFAULT_SOFTNESS = 0.05F;
    public static final int DEFAULT_CHAIN_LENGTH = 0;
    public static final boolean DEFAULT_TIP_ROTATION = false;
    public static final boolean DEFAULT_STRETCH = false;

    /**
     * One IK constraint, modeled after Blender: it lives on the {@code tip}
     * bone, reaches {@code target}, spans {@code chainLength} bones up the
     * hierarchy ({@code 0} = up to the root). When {@code pole} is on, the bend is
     * aimed at {@code poleTarget} (a bone the limb keeps pointing its elbow
     * towards) and {@code poleAngle} (degrees) rolls the bend about the
     * root-to-goal line — Blender's pole angle; with no pole target the bend
     * side comes from the pose (and the authored rest bend on a straight limb).
     * With {@code tipRotation} on, the tip bone copies the {@code target}
     * controller's orientation (Blender's "use tip rotation") instead of
     * keeping its FK pose. With {@code stretch} on, bones that allow it (see
     * {@link JointDoF#stretch}) may lengthen so the chain can reach past its
     * rest length.
     */
    public record Chain(String tip, String target, int chainLength, boolean pole, String poleTarget, float poleAngle, float softness, float weight, boolean enabled, boolean tipRotation, boolean stretch)
    {
        public Chain
        {
            tip = tip == null ? "" : tip;
            target = target == null ? "" : target;
            poleTarget = poleTarget == null ? "" : poleTarget;
            chainLength = Math.max(0, chainLength);
            softness = clamp01(softness);
            weight = clamp01(weight);
        }

        private static float clamp01(float value)
        {
            if (value < 0F)
            {
                return 0F;
            }

            return Math.min(value, 1F);
        }
    }

    /**
     * Per-bone joint freedom for the IK solve — Blender's bone IK panel. Per
     * axis: {@code lock} removes the axis from the solve entirely (it stays
     * frozen at its FK value, so an authored twist survives); {@code limit}
     * clamps the CHANNEL angle into [min, max] degrees — the same numbers the
     * animator sees on the rotation pads; {@code stiffness} 0..1 makes the axis
     * increasingly reluctant to move, shifting the bend to freer joints. One
     * entry per bone of the MODEL — a bone shared by several chains has one
     * set of joints, like a Blender pose bone.
     *
     * <p>{@code stretch} 0..1 is the same idea for LENGTH — how willingly this
     * bone lengthens when its chain has stretching on, {@code 0} pinning it to
     * its natural length — and {@code stretchMax} caps how far it may go, as a
     * fraction of that length. Both only ever matter on a chain whose
     * {@link Chain#stretch} is on.
     */
    public record JointDoF(boolean lockX, boolean lockY, boolean lockZ,
                           boolean limitX, float minX, float maxX,
                           boolean limitY, float minY, float maxY,
                           boolean limitZ, float minZ, float maxZ,
                           float stiffnessX, float stiffnessY, float stiffnessZ,
                           float stretch, float stretchMax)
    {
        public static final float DEFAULT_MIN = -180F;
        public static final float DEFAULT_MAX = 180F;

        /** Every bone stretches equally by default, so ticking a chain's stretch on just works. */
        public static final float DEFAULT_STRETCH = 1F;

        /** Half again as long — enough to cover a reach, small enough that a cubic seam stays weldable. */
        public static final float DEFAULT_STRETCH_MAX = 0.5F;

        public static final JointDoF FREE = new JointDoF(false, false, false,
            false, DEFAULT_MIN, DEFAULT_MAX,
            false, DEFAULT_MIN, DEFAULT_MAX,
            false, DEFAULT_MIN, DEFAULT_MAX,
            0F, 0F, 0F,
            DEFAULT_STRETCH, DEFAULT_STRETCH_MAX);

        public JointDoF
        {
            stiffnessX = Chain.clamp01(stiffnessX);
            stiffnessY = Chain.clamp01(stiffnessY);
            stiffnessZ = Chain.clamp01(stiffnessZ);
            stretch = Chain.clamp01(stretch);
            stretchMax = Math.max(0F, stretchMax);
        }

        /** A free joint carries no information and is not serialized. */
        public boolean isFree()
        {
            return !this.lockX && !this.lockY && !this.lockZ
                && !this.limitX && !this.limitY && !this.limitZ
                && this.stiffnessX <= 0F && this.stiffnessY <= 0F && this.stiffnessZ <= 0F
                && this.stretch == DEFAULT_STRETCH && this.stretchMax == DEFAULT_STRETCH_MAX;
        }
    }
}
