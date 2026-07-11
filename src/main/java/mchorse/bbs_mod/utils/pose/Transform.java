package mchorse.bbs_mod.utils.pose;

import mchorse.bbs_mod.data.DataStorageUtils;
import mchorse.bbs_mod.data.IMapSerializable;
import mchorse.bbs_mod.data.types.MapType;
import mchorse.bbs_mod.utils.MathUtils;
import mchorse.bbs_mod.utils.interps.AutoBezier;
import mchorse.bbs_mod.utils.interps.IInterp;
import mchorse.bbs_mod.utils.joml.Matrices;
import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

public class Transform implements IMapSerializable
{
    private static final Vector3f DEFAULT_SCALE = new Vector3f(1F, 1F, 1F);

    public static final Transform DEFAULT = new Transform();

    public final Vector3f translate = new Vector3f();
    public final Vector3f scale = new Vector3f(DEFAULT_SCALE);

    /**
     * How this transform stores its rotation, Blender-style per-bone
     * {@code rotation_mode}. {@link RotationMode#EULER} uses {@link #rotate} /
     * {@link #rotate2} (keeps &gt;360° spins and per-component curves);
     * {@link RotationMode#QUATERNION} uses {@link #quat} (no gimbal lock). The
     * default is EULER, so untouched data and old scenes behave exactly as
     * before. {@link #createRotation()} is the single point that reads the right
     * one, so every consumer (render, gizmo, IK, keyframes) follows the mode
     * without knowing about it.
     */
    public RotationMode rotationMode = RotationMode.EULER;

    public final Vector3f rotate = new Vector3f();
    public final Vector3f rotate2 = new Vector3f();

    /** The rotation when {@link #rotationMode} is {@link RotationMode#QUATERNION}; identity otherwise. */
    public final Quaternionf quat = new Quaternionf();

    public void lerp(Transform transform, float a)
    {
        this.translate.lerp(transform.translate, a);
        this.scale.lerp(transform.scale, a);
        this.rotate.lerp(transform.rotate, a);
        this.rotate2.lerp(transform.rotate2, a);
    }

    public void lerp(Transform preA, Transform a, Transform b, Transform postB, IInterp interp, float x)
    {
        this.lerp(this.translate, preA.translate, a.translate, b.translate, postB.translate, interp, x);
        this.lerp(this.scale, preA.scale, a.scale, b.scale, postB.scale, interp, x);
        this.lerp(this.rotate, preA.rotate, a.rotate, b.rotate, postB.rotate, interp, x);
        this.lerp(this.rotate2, preA.rotate2, a.rotate2, b.rotate2, postB.rotate2, interp, x);
    }

    private void lerp(Vector3f target, Vector3f preA, Vector3f a, Vector3f b, Vector3f postB, IInterp interp, float x)
    {
        target.x = (float) interp.interpolate(IInterp.context.set(preA.x, a.x, b.x, postB.x, x));
        target.y = (float) interp.interpolate(IInterp.context.set(preA.y, a.y, b.y, postB.y, x));
        target.z = (float) interp.interpolate(IInterp.context.set(preA.z, a.z, b.z, postB.z, x));
    }

    public void autoLerp(Transform preA, Transform a, Transform b, Transform postB, float pt, float at, float bt, float qt, boolean clamped, float x)
    {
        this.autoLerp(this.translate, preA.translate, a.translate, b.translate, postB.translate, pt, at, bt, qt, clamped, x);
        this.autoLerp(this.scale, preA.scale, a.scale, b.scale, postB.scale, pt, at, bt, qt, clamped, x);
        this.autoLerp(this.rotate, preA.rotate, a.rotate, b.rotate, postB.rotate, pt, at, bt, qt, clamped, x);
        this.autoLerp(this.rotate2, preA.rotate2, a.rotate2, b.rotate2, postB.rotate2, pt, at, bt, qt, clamped, x);
    }

    private void autoLerp(Vector3f target, Vector3f preA, Vector3f a, Vector3f b, Vector3f postB, float pt, float at, float bt, float qt, boolean clamped, float x)
    {
        target.x = (float) AutoBezier.get(preA.x, a.x, b.x, postB.x, pt, at, bt, qt, clamped, x);
        target.y = (float) AutoBezier.get(preA.y, a.y, b.y, postB.y, pt, at, bt, qt, clamped, x);
        target.z = (float) AutoBezier.get(preA.z, a.z, b.z, postB.z, pt, at, bt, qt, clamped, x);
    }

    public void add(Transform transform)
    {
        this.translate.add(transform.translate);
        this.scale.mul(transform.scale);
        this.rotate.add(transform.rotate);
        this.rotate2.add(transform.rotate2);
    }

    public void identity()
    {
        this.translate.set(0, 0, 0);
        this.scale.set(1, 1, 1);
        this.rotate.set(0, 0, 0);
        this.rotate2.set(0, 0, 0);
        this.quat.identity();
        this.rotationMode = RotationMode.EULER;
    }

    /**
     * THE channel mirror convention: reflect this transform across the model's
     * YZ symmetry plane. For the ZYX euler order this is exactly
     * {@code D·R·D} with {@code D = diag(-1,1,1)} — translate.x flips sign,
     * rotate/rotate2 keep X and flip Y/Z, scale is untouched. An involution
     * (applying it twice is a no-op), which the mirror-edit fan-out relies on.
     * Every mirror/flip of pose channels goes through here so the negation
     * pattern can't fork between call sites again.
     */
    public void mirrorX()
    {
        this.translate.mul(-1F, 1F, 1F);
        this.rotate.mul(1F, -1F, -1F);
        this.rotate2.mul(1F, -1F, -1F);
    }

    /**
     * THE full local rotation of this transform, radians — the single point every
     * consumer reads, so the storage mode stays invisible to them. In quaternion
     * mode it is {@link #quat}; in euler mode it is {@code ZYX(rotate) · ZYX(rotate2)}.
     */
    public Quaternionf createRotation()
    {
        return this.rotationMode == RotationMode.QUATERNION
            ? new Quaternionf(this.quat)
            : Matrices.toLocalRotationZYXRadians(this.rotate, this.rotate2);
    }

    /**
     * Switch to quaternion storage, folding the current euler stacks into
     * {@link #quat}. A no-op if already in quaternion mode.
     */
    public void setModeQuaternion()
    {
        if (this.rotationMode != RotationMode.QUATERNION)
        {
            this.quat.set(Matrices.toLocalRotationZYXRadians(this.rotate, this.rotate2));
            this.rotationMode = RotationMode.QUATERNION;
        }
    }

    /**
     * Switch to euler storage, decomposing the current quaternion into
     * {@link #rotate} (ZYX) and clearing {@link #rotate2}. A no-op if already in
     * euler mode.
     */
    public void setModeEuler()
    {
        if (this.rotationMode != RotationMode.EULER)
        {
            new Quaternionf(this.quat).normalize().getEulerAnglesZYX(this.rotate);
            this.rotate2.set(0F, 0F, 0F);
            this.rotationMode = RotationMode.EULER;
        }
    }

    public Matrix3f createRotationMatrix()
    {
        return new Matrix3f().rotation(this.createRotation());
    }

    public Matrix4f createMatrix()
    {
        return this.setupMatrix(new Matrix4f());
    }

    /**
     * THE local matrix of a transform's channels, appended onto {@code matrix}:
     * {@code translate · rotation · scale}, with the rotation composed in the
     * one shared place ({@link Matrices#toLocalRotationZYXRadians}). Every
     * pose/slot consumer builds through here, so the composition order can't
     * fork between call sites.
     */
    public Matrix4f setupMatrix(Matrix4f matrix)
    {
        matrix.translate(this.translate);
        matrix.rotate(this.createRotation());
        matrix.scale(this.scale);

        return matrix;
    }

    @Override
    public boolean equals(Object obj)
    {
        if (super.equals(obj))
        {
            return true;
        }

        if (obj instanceof Transform)
        {
            Transform transform = (Transform) obj;

            if (this.rotationMode != transform.rotationMode
                || !this.translate.equals(transform.translate)
                || !this.scale.equals(transform.scale))
            {
                return false;
            }

            return this.rotationMode == RotationMode.QUATERNION
                ? this.quat.equals(transform.quat)
                : this.rotate.equals(transform.rotate) && this.rotate2.equals(transform.rotate2);
        }

        return false;
    }

    public Transform copy()
    {
        Transform transform = new Transform();

        transform.copy(this);

        return transform;
    }

    public void copy(Transform transform)
    {
        this.translate.set(transform.translate);
        this.scale.set(transform.scale);
        this.rotate.set(transform.rotate);
        this.rotate2.set(transform.rotate2);
        this.quat.set(transform.quat);
        this.rotationMode = transform.rotationMode;
    }

    public boolean isDefault()
    {
        return this.equals(DEFAULT);
    }

    public void toRad()
    {
        this.rotate.x = MathUtils.toRad(this.rotate.x);
        this.rotate.y = MathUtils.toRad(this.rotate.y);
        this.rotate.z = MathUtils.toRad(this.rotate.z);
        this.rotate2.x = MathUtils.toRad(this.rotate2.x);
        this.rotate2.y = MathUtils.toRad(this.rotate2.y);
        this.rotate2.z = MathUtils.toRad(this.rotate2.z);
    }

    public void toDeg()
    {
        this.rotate.x = MathUtils.toDeg(this.rotate.x);
        this.rotate.y = MathUtils.toDeg(this.rotate.y);
        this.rotate.z = MathUtils.toDeg(this.rotate.z);
        this.rotate2.x = MathUtils.toDeg(this.rotate2.x);
        this.rotate2.y = MathUtils.toDeg(this.rotate2.y);
        this.rotate2.z = MathUtils.toDeg(this.rotate2.z);
    }

    @Override
    public void toData(MapType data)
    {
        if (!this.isDefault())
        {
            data.put("t", DataStorageUtils.vector3fToData(this.translate));
            data.put("s", DataStorageUtils.vector3fToData(this.scale));

            /* The presence of "q" is itself the mode discriminator, so no separate
             * flag is needed and old scenes (only r/r2) read back as EULER. */
            if (this.rotationMode == RotationMode.QUATERNION)
            {
                data.put("q", DataStorageUtils.quaternionfToData(this.quat));
            }
            else
            {
                data.put("r", DataStorageUtils.vector3fToData(this.rotate));
                data.put("r2", DataStorageUtils.vector3fToData(this.rotate2));
            }
        }
    }

    @Override
    public void fromData(MapType data)
    {
        this.identity();

        this.translate.set(DataStorageUtils.vector3fFromData(data.getList("t")));
        this.scale.set(DataStorageUtils.vector3fFromData(data.getList("s"), DEFAULT_SCALE));

        if (data.has("q"))
        {
            this.rotationMode = RotationMode.QUATERNION;
            this.quat.set(DataStorageUtils.quaternionfFromData(data.getList("q")));
        }
        else
        {
            this.rotationMode = RotationMode.EULER;
            this.rotate.set(DataStorageUtils.vector3fFromData(data.getList("r")));
            this.rotate2.set(DataStorageUtils.vector3fFromData(data.getList("r2")));
        }
    }

    /** Per-transform rotation storage, Blender's {@code rotation_mode} (Euler | Quaternion). */
    public enum RotationMode
    {
        EULER, QUATERNION
    }
}