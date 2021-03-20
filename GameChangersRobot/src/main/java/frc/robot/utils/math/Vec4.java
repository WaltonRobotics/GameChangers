package frc.robot.utils.math;

public class Vec4 {

    public double x, y, z, w;

    public Vec4() {
        this(0.0);
    }

    public Vec4(double scalar) {
        this(scalar, scalar, scalar, scalar);
    }

    public Vec4(Vec3 xyz, double w) {
        this(xyz.x, xyz.y, xyz.z, w);
    }

    public Vec4(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Vec4 add(Vec4 other)
    {
        Vec4 result = new Vec4();

        result.x += other.x;
        result.y += other.y;
        result.z += other.z;
        result.w += other.w;

        return result;
    }

    public Vec4 subtract(Vec4 other)
    {
        Vec4 result = new Vec4();

        result.x -= other.x;
        result.y -= other.y;
        result.z -= other.z;
        result.w -= other.w;

        return result;
    }

    public Vec4 multiply(Vec4 other)
    {
        Vec4 result = new Vec4();

        result.x *= other.x;
        result.y *= other.y;
        result.z *= other.z;
        result.w *= other.w;

        return result;
    }

    public Vec4 divide(Vec4 other)
    {
        Vec4 result = new Vec4();

        result.x /= other.x;
        result.y /= other.y;
        result.z /= other.z;
        result.w /= other.w;

        return result;
    }

    public Vec4 multiply(Mat4 transform)
    {
        return new Vec4(
                transform.getRows()[0].x * x + transform.getRows()[0].y * y
                        + transform.getRows()[0].z * z + transform.getRows()[0].w * w,
                transform.getRows()[1].x * x + transform.getRows()[1].y * y
                        + transform.getRows()[1].z * z + transform.getRows()[1].w * w,
                transform.getRows()[2].x * x + transform.getRows()[2].y * y
                        + transform.getRows()[2].z * z + transform.getRows()[2].w * w,
                transform.getRows()[3].x * x + transform.getRows()[3].y * y
                        + transform.getRows()[3].z * z + transform.getRows()[3].w * w
        );
    }

    public double dot(Vec4 other)
    {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Vec4)) return false;
        Vec4 vec4 = (Vec4) object;
        return vec4.x == x && vec4.y == y && vec4.z == z && vec4.w == w;
    }

    @Override
    public String toString() {
        return "vec4: (" + x + ", " + y + ", " + z + ", " + w + ")";
    }
}
