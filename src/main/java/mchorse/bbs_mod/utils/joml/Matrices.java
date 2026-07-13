package mchorse.bbs_mod.utils.joml;

import org.joml.Matrix3d;
import org.joml.Matrix3f;
import org.joml.Matrix4d;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

public class Matrices
{
    public static final Matrix3f EMPTY_3F = new Matrix3f();
    public static final Matrix3d EMPTY_3D = new Matrix3d();
    public static final Matrix4f EMPTY_4F = new Matrix4f();
    public static final Matrix4d EMPTY_4D = new Matrix4d();

    /* Temporary matrices that can be used to avoid allocations */
    public static final Matrix3f TEMP_3F = new Matrix3f();
    public static final Matrix3d TEMP_3D = new Matrix3d();
    public static final Matrix4f TEMP_4F = new Matrix4f();
    public static final Matrix4d TEMP_4D = new Matrix4d();

    private static final Matrix3f rotation = new Matrix3f();
    private static final Vector3f forward = new Vector3f();

    private static final Matrix3f lerpA = new Matrix3f();
    private static final Matrix3f lerpB = new Matrix3f();
    private static final Quaternionf lerpQa = new Quaternionf();
    private static final Quaternionf lerpQb = new Quaternionf();
    private static final Vector3f lerpVa = new Vector3f();
    private static final Vector3f lerpVb = new Vector3f();

    public static Vector3f rotate(Vector3f vector, float pitch, float yaw)
    {
        rotation.identity();
        rotation.rotateY(yaw);
        rotation.rotateX(pitch);
        rotation.transform(vector);

        return vector;
    }

    public static Vector3f rotation(float pitch, float yaw)
    {
        return rotate(forward.set(0, 0, 1), pitch, yaw);
    }

    public static Matrix3f direction(Vector3f forward)
    {
        Matrix3f direction = new Matrix3f();
        Vector3f right = new Vector3f(0, 1, 0);
        Vector3f up = new Vector3f(forward);

        if (right.equals(forward))
        {
            right.set(1, 0, 0);
        }

        right.cross(forward);
        up.cross(right);

        direction.m00 = right.x;
        direction.m01 = right.y;
        direction.m02 = right.z;
        direction.m10 = forward.x;
        direction.m11 = forward.y;
        direction.m12 = forward.z;
        direction.m20 = up.x;
        direction.m21 = up.y;
        direction.m22 = up.z;

        return direction;
    }

    public static Matrix4f lerp(Matrix4f a, Matrix4f b, float t)
    {
        return lerp(a, b, t, new Matrix4f());
    }

    public static Matrix4f lerp(Matrix4f a, Matrix4f b, float t, Matrix4f dest)
    {
        Quaternionf q1 = lerpQa.setFromNormalized(lerpA.set(a));
        Quaternionf q2 = lerpQb.setFromNormalized(lerpB.set(b));

        q1.slerp(q2, t);

        dest.identity().rotate(q1);
        dest.setTranslation(a.getTranslation(lerpVa).lerp(b.getTranslation(lerpVb), t));

        return dest;
    }

    public static String toString(Matrix3f m)
    {
        return m.m00() + ", " + m.m10() + ", " + m.m20() + "\n" +
            m.m01() + ", " + m.m11() + ", " + m.m21() + "\n" +
            m.m02() + ", " + m.m12() + ", " + m.m22();
    }

    public static String toString(Matrix4f m)
    {
        return m.m00() + ", " + m.m10() + ", " + m.m20() + ", " + m.m30() + "\n" +
            m.m01() + ", " + m.m11() + ", " + m.m21() + ", " + m.m31() + "\n" +
            m.m02() + ", " + m.m12() + ", " + m.m22() + ", " + m.m32() + "\n" +
            m.m03() + ", " + m.m13() + ", " + m.m23() + ", " + m.m33() + "\n";
    }

    public static Quaternionf toQuaternionZYXDegrees(float xDeg, float yDeg, float zDeg)
    {
        float x = (float) (xDeg * Math.PI / 180.0);
        float y = (float) (yDeg * Math.PI / 180.0);
        float z = (float) (zDeg * Math.PI / 180.0);

        return new Quaternionf().rotationZYX(z, y, x);
    }

    public static Quaternionf toQuaternionZYXRadians(float x, float y, float z)
    {
        return new Quaternionf().rotationZYX(z, y, x);
    }

    public static Vector3f toEulerZYXDegrees(Quaternionf q)
    {
        Vector3f radZYX = new Vector3f();

        new Quaternionf(q).normalize().getEulerAnglesZYX(radZYX);

        return radZYX.mul((float) (180.0 / Math.PI));
    }

    /**
     * ZYX euler angles (radians) of {@code rotation}, resolved to the
     * representation continuous with {@code referenceRadians} — written into
     * {@code dest}.
     *
     * <p>Every orientation has TWO ZYX euler solutions: the principal one
     * {@code (x, y, z)} and its flip {@code (x+π, π-y, z+π)} (plus whole-turn
     * shifts of each). A plain component-wise unwrap slides within ONE branch
     * but can never cross to the other, so a decomposition whose middle angle
     * passes the ±90° pole can snap to the flipped branch — spilling π into the
     * outer two angles though the orientation barely moved (e.g. a world/parent-
     * space Y-turn then shows X and Z jumping too). This picks whichever branch,
     * unwrapped to the reference, lands nearest, which is the branch the value
     * was already on. It is Blender's {@code mat3_to_compatible_eulO}, and is THE
     * matrix/quaternion&rarr;euler readback for any channel that must stay
     * continuous with its previous value (IK/pose blending, multi-bone pivot,
     * the gizmo's spatial rotations).
     */
    public static Vector3f toCompatibleEulerZYXRadians(Quaternionf rotation, Vector3f referenceRadians, Vector3f dest)
    {
        Vector3f raw = new Quaternionf(rotation).normalize().getEulerAnglesZYX(new Vector3f());

        return compatibleEulerZYX(raw, referenceRadians, (float) Math.PI, dest);
    }

    /** See {@link #toCompatibleEulerZYXRadians(Quaternionf, Vector3f, Vector3f)}; from a rotation matrix. */
    public static Vector3f toCompatibleEulerZYXRadians(Matrix3f rotation, Vector3f referenceRadians, Vector3f dest)
    {
        return compatibleEulerZYX(rotation.getEulerAnglesZYX(new Vector3f()), referenceRadians, (float) Math.PI, dest);
    }

    /** See {@link #toCompatibleEulerZYXRadians(Quaternionf, Vector3f, Vector3f)}; angles in degrees. */
    public static Vector3f toCompatibleEulerZYXDegrees(Quaternionf rotation, Vector3f referenceDegrees, Vector3f dest)
    {
        Vector3f raw = new Quaternionf(rotation).normalize().getEulerAnglesZYX(new Vector3f()).mul((float) (180.0 / Math.PI));

        return compatibleEulerZYX(raw, referenceDegrees, 180F, dest);
    }

    /**
     * Choose the ZYX euler branch nearest {@code reference}: the principal
     * solution {@code raw} or its flip {@code (x+half, half-y, z+half)}, each
     * pulled by whole turns to the reference, whichever is closer by total
     * angular offset. {@code halfTurn} is π (radians) or 180 (degrees).
     */
    private static Vector3f compatibleEulerZYX(Vector3f raw, Vector3f reference, float halfTurn, Vector3f dest)
    {
        float fullTurn = halfTurn * 2F;

        float ax = unwrapNear(raw.x, reference.x, fullTurn);
        float ay = unwrapNear(raw.y, reference.y, fullTurn);
        float az = unwrapNear(raw.z, reference.z, fullTurn);

        float bx = unwrapNear(raw.x + halfTurn, reference.x, fullTurn);
        float by = unwrapNear(halfTurn - raw.y, reference.y, fullTurn);
        float bz = unwrapNear(raw.z + halfTurn, reference.z, fullTurn);

        boolean flip = offsetL1(bx, by, bz, reference) < offsetL1(ax, ay, az, reference);

        return flip ? dest.set(bx, by, bz) : dest.set(ax, ay, az);
    }

    /** {@code value} shifted by whole {@code period}s to sit nearest {@code reference}. */
    private static float unwrapNear(float value, float reference, float period)
    {
        return value + Math.round((reference - value) / period) * period;
    }

    /** Total absolute per-axis offset of an euler triple from {@code reference}. */
    private static float offsetL1(float x, float y, float z, Vector3f reference)
    {
        return Math.abs(x - reference.x) + Math.abs(y - reference.y) + Math.abs(z - reference.z);
    }

    /**
     * A transform's euler rotation ({@code ZYX(rotate)}) as a quaternion — the
     * renderer's order. Every consumer of a bone's euler stack (bone renderers,
     * orient seeds, IK blending, the pose matrix) goes through these two methods,
     * so the convention can never fork. Degrees flavour for cubic model channels,
     * radians for poses and BOBJ bones.
     */
    public static Quaternionf toLocalRotationZYXDegrees(Vector3f rotateDeg)
    {
        return toQuaternionZYXDegrees(rotateDeg.x, rotateDeg.y, rotateDeg.z);
    }

    /** See {@link #toLocalRotationZYXDegrees}; angles in radians. */
    public static Quaternionf toLocalRotationZYXRadians(Vector3f rotate)
    {
        return toQuaternionZYXRadians(rotate.x, rotate.y, rotate.z);
    }

    /**
     * Conjugate a rotation expressed in the cubic model's X-mirrored space back
     * to the unmirrored frame: {@code D·R·D} with {@code D = diag(-1,1,1)}. The
     * same YZ-plane mirror convention as
     * {@link mchorse.bbs_mod.utils.pose.Transform#mirrorX()} applies to the
     * channels — this is its matrix-level counterpart for the IK math.
     */
    private static Quaternionf unmirrorX(Matrix3f rotationMirrored)
    {
        Matrix3f mirror = new Matrix3f().scaling(-1F, 1F, 1F);
        Matrix3f rot = new Matrix3f(mirror).mul(rotationMirrored).mul(mirror);

        return new Quaternionf().setFromNormalized(rot);
    }

    public static Quaternionf fromToMirroredX(Vector3f restDirLocal, Vector3f desiredDirLocal)
    {
        Vector3f restM = new Vector3f(restDirLocal);
        Vector3f desM = new Vector3f(desiredDirLocal);

        restM.x = -restM.x;
        desM.x = -desM.x;

        restM.normalize();
        desM.normalize();

        Quaternionf qMir = new Quaternionf().rotationTo(restM, desM);

        return unmirrorX(new Matrix3f().set(qMir));
    }

    /**
     * The local rotation taking the rest frame {@code (restDir, restNormal)} to the
     * target frame {@code (toDir, toNormal)}, where each frame is an orthonormal
     * basis with its first axis along the direction and its roll fixed by the normal.
     * Unlike {@link #fromToMirroredX} — which only aligns the directions and picks the
     * shortest arc for the roll (so it flips at a 180° swing) — the normal pins the
     * roll, so the result never flips and carries a defined twist (the IK bend plane).
     *
     * <p>Both frames are built in the cubic model's X-mirrored space and the result is
     * conjugated back, exactly as {@link #fromToMirroredX} does, so the cubic geometry
     * bends the correct way. With {@code toDir == restDir} and {@code toNormal} sharing
     * {@code restNormal}'s plane the result is identity (no baseline twist at rest).
     */
    public static Quaternionf orientMirroredX(Vector3f restDir, Vector3f restNormal, Vector3f toDir, Vector3f toNormal)
    {
        Matrix3f rest = mirroredFrame(restDir, restNormal);
        Matrix3f to = mirroredFrame(toDir, toNormal);

        return unmirrorX(to.mul(rest.transpose()));
    }

    /**
     * Orthonormal right-handed basis in X-mirrored space: first column along the
     * mirrored direction, the mirrored normal fixing the roll (its perpendicular
     * component). Falls back to an arbitrary perpendicular when the normal is parallel
     * to the direction (degenerate roll).
     */
    private static Matrix3f mirroredFrame(Vector3f dir, Vector3f normal)
    {
        Vector3f u = new Vector3f(-dir.x, dir.y, dir.z).normalize();
        Vector3f w = new Vector3f(u).cross(-normal.x, normal.y, normal.z);

        if (w.lengthSquared() < 1.0e-12f)
        {
            Vector3f ref = Math.abs(u.x) < 0.9F ? new Vector3f(1F, 0F, 0F) : new Vector3f(0F, 1F, 0F);

            w.set(u).cross(ref);
        }

        w.normalize();

        Vector3f v = new Vector3f(w).cross(u);
        Matrix3f m = new Matrix3f();

        m.m00 = u.x; m.m01 = u.y; m.m02 = u.z;
        m.m10 = v.x; m.m11 = v.y; m.m12 = v.z;
        m.m20 = w.x; m.m21 = w.y; m.m22 = w.z;

        return m;
    }

    /**
     * Extracts the component of {@code q} that rotates around {@code axis}
     * (the swing-twist decomposition's twist part). {@code axis} must be
     * normalized. Returns identity when the twist is undefined (180° swing).
     */
    public static Quaternionf twistAbout(Quaternionf q, Vector3f axis)
    {
        float dot = q.x * axis.x + q.y * axis.y + q.z * axis.z;
        Quaternionf twist = new Quaternionf(axis.x * dot, axis.y * dot, axis.z * dot, q.w);

        if (twist.lengthSquared() < 1.0e-12f)
        {
            return new Quaternionf();
        }

        return twist.normalize();
    }
}
