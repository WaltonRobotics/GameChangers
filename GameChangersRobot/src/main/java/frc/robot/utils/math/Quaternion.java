package frc.robot.utils.math;

import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

public class Quaternion {

    public double x, y, z, w;

    public Quaternion() {
        this(0, 0, 0, 1);
    }

    public Quaternion(Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
    }

    public Quaternion(Vec4 vec) {
        this(vec.x, vec.y, vec.z, vec.w);
    }

    public Quaternion(double scalar) {
        this(scalar, scalar, scalar, scalar);
    }

    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Quaternion(Vec3 xyz, double w) {
        this.setXYZ(xyz);
        this.w = w;
    }

    public static Quaternion identity() {
        return new Quaternion(0.0, 0.0, 0.0, 1.0);
    }

    public static Quaternion fromEulerAngles(Vec3 angles) {
        Quaternion pitch = new Quaternion(new Vec3(1.0, 0.0, 0.0), angles.x);
        Quaternion yaw = new Quaternion(new Vec3(0.0, 1.0, 0.0), angles.y);
        Quaternion roll = new Quaternion(new Vec3(0.0, 0.0, 1.0), angles.z);
        return pitch.multiply(yaw).multiply(roll);
    }

    public static Quaternion rotation(Vec3 unitVec0, Vec3 unitVec1) {
        double cosHalfAngleX2, recipCosHalfAngleX2;
        cosHalfAngleX2 = StrictMath.sqrt((2.0f * (1.0f + unitVec0.dot(unitVec1))));
        recipCosHalfAngleX2 = (1.0f / cosHalfAngleX2);
        return new Quaternion((unitVec0.cross(unitVec1).multiply(recipCosHalfAngleX2)), (cosHalfAngleX2 * 0.5));
    }

    public static Quaternion rotation(double radians, Vec3 unitVec) {
        double angle = radians * 0.5f;
        return new Quaternion((unitVec.multiply(StrictMath.sin(angle))), StrictMath.cos(angle));
    }

    public static Vec3 rotate(Quaternion quat, Vec3 vec) {
        double tmpX, tmpY, tmpZ, tmpW;
        tmpX = (((quat.w * vec.x) + (quat.y * vec.z)) - (quat.z * vec.y));
        tmpY = (((quat.w * vec.y) + (quat.z * vec.x)) - (quat.x * vec.z));
        tmpZ = (((quat.w * vec.z) + (quat.x * vec.y)) - (quat.y * vec.x));
        tmpW = (((quat.x * vec.x) + (quat.y * vec.y)) + (quat.z * vec.z));
        return new Vec3(
                ((((tmpW * quat.x) + (tmpX * quat.w)) - (tmpY * quat.z)) + (tmpZ * quat.y)),
                ((((tmpW * quat.y) + (tmpY * quat.w)) - (tmpZ * quat.x)) + (tmpX * quat.z)),
                ((((tmpW * quat.z) + (tmpZ * quat.w)) - (tmpX * quat.y)) + (tmpY * quat.x))
        );
    }

    private Vec3 twoAxisRot(double r11, double r12, double r21, double r31, double r32) {
        Vec3 result = new Vec3();

        result.x = StrictMath.atan2(r11, r12);
        result.y = StrictMath.acos(r21);
        result.z = StrictMath.atan2(r31, r32);

        return result;
    }

    private Vec3 threeAxisRot(double r11, double r12, double r21, double r31, double r32) {
        Vec3 result = new Vec3();

        result.x = StrictMath.atan2(r31, r32);
        result.y = StrictMath.asin(r21);
        result.z = StrictMath.atan2(r11, r12);

        return result;
    }

    public Vec3 toEulerAngles(EulerSequence rotSeq) {
        switch (rotSeq) {
            case ZYX:
                return threeAxisRot(2 * (x * y + w * z),
                        w * w + x * x - y * y - z * z,
                        -2 * (x * z - w * y),
                        2 * (y * z + w * x),
                        w * w - x * x - y * y + z * z
                );

            case ZYZ:
                return twoAxisRot(2 * (y * z - w * x),
                        2 * (x * z + w * y),
                        w * w - x * x - y * y + z * z,
                        2 * (y * z + w * x),
                        -2 * (x * z - w * y)
                );

            case ZXY:
                return threeAxisRot(-2 * (x * y - w * z),
                        w * w - x * x + y * y - z * z,
                        2 * (y * z + w * x),
                        -2 * (x * z - w * y),
                        w * w - x * x - y * y + z * z
                );

            case ZXZ:
                return twoAxisRot(2 * (x * z + w * y),
                        -2 * (y * z - w * x),
                        w * w - x * x - y * y + z * z,
                        2 * (x * z - w * y),
                        2 * (y * z + w * x)
                );

            case YXZ:
                return threeAxisRot(2 * (x * z + w * y),
                        w * w - x * x - y * y + z * z,
                        -2 * (y * z - w * x),
                        2 * (x * y + w * z),
                        w * w - x * x + y * y - z * z
                );

            case YXY:
                return twoAxisRot(2 * (x * y - w * z),
                        2 * (y * z + w * x),
                        w * w - x * x + y * y - z * z,
                        2 * (x * y + w * z),
                        -2 * (y * z - w * x)
                );

            case YZX:
                return threeAxisRot(-2 * (x * z - w * y),
                        w * w + x * x - y * y - z * z,
                        2 * (x * y + w * z),
                        -2 * (y * z - w * x),
                        w * w - x * x + y * y - z * z
                );

            case YZY:
                return twoAxisRot(2 * (y * z + w * x),
                        -2 * (x * y - w * z),
                        w * w - x * x + y * y - z * z,
                        2 * (y * z - w * x),
                        2 * (x * y + w * z)
                );

            case XYZ:
                return threeAxisRot(-2 * (y * z - w * x),
                        w * w - x * x - y * y + z * z,
                        2 * (x * z + w * y),
                        -2 * (x * y - w * z),
                        w * w + x * x - y * y - z * z
                );

            case XYX:
                return twoAxisRot(2 * (x * y + w * z),
                        -2 * (x * z - w * y),
                        w * w + x * x - y * y - z * z,
                        2 * (x * y - w * z),
                        2 * (x * z + w * y)
                );

            case XZY:
                return threeAxisRot(2 * (y * z + w * x),
                        w * w - x * x + y * y - z * z,
                        -2 * (x * y - w * z),
                        2 * (x * z + w * y),
                        w * w + x * x - y * y - z * z
                );

            case XZX:
                return twoAxisRot(2 * (x * z - w * y),
                        2 * (x * y + w * z),
                        w * w + x * x - y * y - z * z,
                        2 * (x * z + w * y),
                        -2 * (x * y - w * z)
                );

            default:
                DebuggingLog.getInstance().getLogger().log(Level.SEVERE, "Unknown Euler sequence + "
                        + rotSeq.name() + " passed into Quaternion toEulerAngles method");
                break;
        }

        return null;
    }

    public Vec3 getXYZ() {
        return new Vec3(x, y, z);
    }

    public Quaternion setXYZ(Vec3 vec) {
        x = vec.x;
        y = vec.y;
        z = vec.z;
        return this;
    }

    public Vec3 getAxis() {
        double x = 1.0 - w * w;
        if (x < 0.0000001) // Divide by zero safety check
            return Vec3.xAxis();

        double x2 = x * x;
        return getXYZ().divide(x2);
    }

    public Quaternion add(Quaternion quaternion) {
        return new Quaternion(x + quaternion.x, y + quaternion.y, z + quaternion.z, w + quaternion.w);
    }

    public Quaternion subtract(Quaternion quaternion) {
        return new Quaternion(x - quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w);
    }

    public Quaternion multiply(double scalar) {
        return new Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    public Quaternion divide(double scalar) {
        return new Quaternion(x / scalar, y / scalar, z / scalar, w / scalar);
    }

    public Quaternion negate() {
        return new Quaternion(-x, -y, -z, -w);
    }

    public double norm(Quaternion quaternion) {
        double result;
        result = (quaternion.x * quaternion.x);
        result = (result + (quaternion.y * quaternion.y));
        result = (result + (quaternion.z * quaternion.z));
        result = (result + (quaternion.w * quaternion.w));
        return result;
    }

    public double length(Quaternion quaternion) {
        return StrictMath.sqrt(norm(quaternion));
    }

    public Quaternion normalize(Quaternion quaternion) {
        double lenSqr, lenInv;
        lenSqr = norm(quaternion);
        lenInv = 1.0 / StrictMath.sqrt(lenSqr);
        return quaternion.multiply(lenInv);
    }

    public Quaternion normalizeEst(Quaternion quaternion) {
        double lenSqr, lenInv;
        lenSqr = norm(quaternion);
        lenInv = 1.0 / StrictMath.sqrt(lenSqr);
        return quaternion.multiply(lenInv);
    }

    public Quaternion multiply(Quaternion quat) {
        return normalize(new Quaternion(
                (((w * quat.x) + (x * quat.w)) + (y * quat.z)) - (z * quat.y),
                (((w * quat.y) + (y * quat.w)) + (z * quat.x)) - (x * quat.z),
                (((w * quat.z) + (z * quat.w)) + (x * quat.y)) - (y * quat.x),
                (((w * quat.w) - (x * quat.x)) - (y * quat.y)) - (z * quat.z)
        ));
    }

    public Quaternion conjugate() {
        return new Quaternion(-x, -y, -z, w);
    }

    public Quaternion select(Quaternion quat0, Quaternion quat1, boolean select1) {
        return new Quaternion(select1 ? quat1.x : quat0.x, select1 ? quat1.y : quat0.y,
                select1 ? quat1.z : quat0.z, select1 ? quat1.w : quat0.w);
    }

    public double dot(Quaternion other) {
        double result = (x * other.x);
        result = (result + (y * other.y));
        result = (result + (z * other.z));
        result = (result + (w * other.w));
        return result;
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Quaternion)) return false;
        Quaternion quaternion = (Quaternion) object;
        return quaternion.x == x && quaternion.y == y && quaternion.z == z && quaternion.w == w;
    }

    @Override
    public String toString() {
        return "Quaternion: (" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    public enum EulerSequence {
        ZYX, ZYZ, ZXY, ZXZ, YXZ, YXY, YZX, YZY, XYZ, XYX, XZY, XZX
    }
}
