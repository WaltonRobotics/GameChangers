package frc.robot.utils.math;

public class Vec3 {

    public double x, y, z;

    public Vec3() {
        this(0.0);
    }

    public Vec3(double scalar) {
        this(scalar, scalar, scalar);
    }

    public Vec3(double x, double y) {
        this(x, y, 0.0);
    }

    public Vec3(Vec4 other) {
        this(other.x, other.y, other.z);
    }

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Vec3 up()
    {
        return new Vec3(0.0f, 1.0f, 0.0f);
    }

    public static Vec3 down()
    {
        return new Vec3(0.0f, -1.0f, 0.0f);
    }

    public static Vec3 left()
    {
        return new Vec3(-1.0f, 0.0f, 0.0f);
    }

    public static Vec3 right()
    {
        return new Vec3(1.0f, 1.0f, 0.0f);
    }

    public static Vec3 zero()
    {
        return new Vec3(0.0f, 0.0f, 0.0f);
    }

    public static Vec3 xAxis()
    {
        return new Vec3(1.0f, 0.0f, 0.0f);
    }

    public static Vec3 yAxis()
    {
        return new Vec3(0.0f, 1.0f, 0.0f);
    }

    public static Vec3 zAxis()
    {
        return new Vec3(0.0f, 0.0f, 1.0f);
    }

    public Vec3 add(Vec3 other)
    {
        Vec3 result = new Vec3();

        result.x += other.x;
        result.y += other.y;
        result.z += other.z;

        return result;
    }

    public Vec3 subtract(Vec3 other)
    {
        Vec3 result = new Vec3();

        result.x -= other.x;
        result.y -= other.y;
        result.z -= other.z;

        return result;
    }

    public Vec3 multiply(Vec3 other)
    {
        Vec3 result = new Vec3();

        result.x *= other.x;
        result.y *= other.y;
        result.z *= other.z;

        return result;
    }

    public Vec3 divide(Vec3 other)
    {
        Vec3 result = new Vec3();

        result.x /= other.x;
        result.y /= other.y;
        result.z /= other.z;

        return result;
    }

    public Vec3 add(double other)
    {
        Vec3 result = new Vec3();

        result.x += other;
        result.y += other;
        result.z += other;

        return result;
    }

    public Vec3 subtract(double other)
    {
        Vec3 result = new Vec3();

        result.x -= other;
        result.y -= other;
        result.z -= other;

        return result;
    }

    public Vec3 multiply(double other)
    {
        Vec3 result = new Vec3();

        result.x *= other;
        result.y *= other;
        result.z *= other;

        return result;
    }

    public Vec3 divide(double other)
    {
        Vec3 result = new Vec3();

        result.x /= other;
        result.y /= other;
        result.z /= other;

        return result;
    }

    public Vec3 multiply(Mat4 transform)
    {
        return new Vec3(
                transform.getRows()[0].x * x + transform.getRows()[0].y * y
                        + transform.getRows()[0].z * z + transform.getRows()[0].w,
                transform.getRows()[1].x * x + transform.getRows()[1].y * y
                        + transform.getRows()[1].z * z + transform.getRows()[1].w,
                transform.getRows()[2].x * x + transform.getRows()[2].y * y
                        + transform.getRows()[2].z * z + transform.getRows()[2].w
        );
    }

    public Vec3 cross(Vec3 other)
    {
        return new Vec3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    public double dot(Vec3 other)
    {
        return x * other.x + y * other.y + z * other.z;
    }

    public double magnitude()
    {
        return StrictMath.sqrt(x * x + y * y + z * z);
    }

    public Vec3 normalize()
    {
        double length = magnitude();
        return new Vec3(x / length, y / length, z / length);
    }

    public double distance(Vec3 other)
    {
        double a = x - other.x;
        double b = y - other.y;
        double c = z - other.z;
        return StrictMath.sqrt(a * a + b * b + c * c);
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Vec3)) return false;
        Vec3 vec3 = (Vec3) object;
        return vec3.x == x && vec3.y == y && vec3.z == z;
    }

    @Override
    public String toString() {
        return "vec3: (" + x + ", " + y + ", " + z + ")";
    }
}
