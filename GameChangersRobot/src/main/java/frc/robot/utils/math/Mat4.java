package frc.robot.utils.math;

import java.util.Arrays;

public class Mat4 {

    // Row major
    // [col + row * 4]
    public double[] elements = new double[4 * 4];

    public Mat4() {
        clear();
    }

    public Mat4(double diagonal) {
        clear();

        elements[0] = diagonal;
        elements[1 + 4] = diagonal;
        elements[2 + 2 * 4] = diagonal;
        elements[3 + 3 * 4] = diagonal;
    }

    public Mat4(double[] elements) {
        System.arraycopy(elements, 0, this.elements, 0, 16);
    }

    public Mat4(Vec4 row0, Vec4 row1, Vec4 row2, Vec4 row3) {
        elements[0] = row0.x;
        elements[1] = row0.y;
        elements[2] = row0.z;
        elements[3] = row0.w;
        elements[4] = row1.x;
        elements[5] = row1.y;
        elements[6] = row1.z;
        elements[7] = row1.w;
        elements[8] = row2.x;
        elements[9] = row2.y;
        elements[10] = row2.z;
        elements[11] = row2.w;
        elements[12] = row3.x;
        elements[13] = row3.y;
        elements[14] = row3.z;
        elements[15] = row3.w;
    }

    public static Mat4 identity() {
        return new Mat4(1.0);
    }

    public Mat4 multiply(Mat4 other)
    {
        double[] data = new double[16];
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                double sum = 0.0;
                for (int e = 0; e < 4; e++)
                {
                    sum += elements[e + row * 4] * other.elements[col + e * 4];
                }
                data[col + row * 4] = sum;
            }
        }
        System.arraycopy(data, 0, this.elements, 0, 16);
        return this;
    }

    public Vec3 multiply(Vec3 other)
    {
        return other.multiply(this);
    }

    public Vec4 multiply(Vec4 other)
    {
        return other.multiply(this);
    }

    public Mat4 getInverse()
    {
        double[] temp = new double[16];

        temp[0] = elements[5] * elements[10] * elements[15] -
                elements[5] * elements[11] * elements[14] -
                elements[9] * elements[6] * elements[15] +
                elements[9] * elements[7] * elements[14] +
                elements[13] * elements[6] * elements[11] -
                elements[13] * elements[7] * elements[10];

        temp[4] = -elements[4] * elements[10] * elements[15] +
                elements[4] * elements[11] * elements[14] +
                elements[8] * elements[6] * elements[15] -
                elements[8] * elements[7] * elements[14] -
                elements[12] * elements[6] * elements[11] +
                elements[12] * elements[7] * elements[10];

        temp[8] = elements[4] * elements[9] * elements[15] -
                elements[4] * elements[11] * elements[13] -
                elements[8] * elements[5] * elements[15] +
                elements[8] * elements[7] * elements[13] +
                elements[12] * elements[5] * elements[11] -
                elements[12] * elements[7] * elements[9];

        temp[12] = -elements[4] * elements[9] * elements[14] +
                elements[4] * elements[10] * elements[13] +
                elements[8] * elements[5] * elements[14] -
                elements[8] * elements[6] * elements[13] -
                elements[12] * elements[5] * elements[10] +
                elements[12] * elements[6] * elements[9];

        temp[1] = -elements[1] * elements[10] * elements[15] +
                elements[1] * elements[11] * elements[14] +
                elements[9] * elements[2] * elements[15] -
                elements[9] * elements[3] * elements[14] -
                elements[13] * elements[2] * elements[11] +
                elements[13] * elements[3] * elements[10];

        temp[5] = elements[0] * elements[10] * elements[15] -
                elements[0] * elements[11] * elements[14] -
                elements[8] * elements[2] * elements[15] +
                elements[8] * elements[3] * elements[14] +
                elements[12] * elements[2] * elements[11] -
                elements[12] * elements[3] * elements[10];

        temp[9] = -elements[0] * elements[9] * elements[15] +
                elements[0] * elements[11] * elements[13] +
                elements[8] * elements[1] * elements[15] -
                elements[8] * elements[3] * elements[13] -
                elements[12] * elements[1] * elements[11] +
                elements[12] * elements[3] * elements[9];

        temp[13] = elements[0] * elements[9] * elements[14] -
                elements[0] * elements[10] * elements[13] -
                elements[8] * elements[1] * elements[14] +
                elements[8] * elements[2] * elements[13] +
                elements[12] * elements[1] * elements[10] -
                elements[12] * elements[2] * elements[9];

        temp[2] = elements[1] * elements[6] * elements[15] -
                elements[1] * elements[7] * elements[14] -
                elements[5] * elements[2] * elements[15] +
                elements[5] * elements[3] * elements[14] +
                elements[13] * elements[2] * elements[7] -
                elements[13] * elements[3] * elements[6];

        temp[6] = -elements[0] * elements[6] * elements[15] +
                elements[0] * elements[7] * elements[14] +
                elements[4] * elements[2] * elements[15] -
                elements[4] * elements[3] * elements[14] -
                elements[12] * elements[2] * elements[7] +
                elements[12] * elements[3] * elements[6];

        temp[10] = elements[0] * elements[5] * elements[15] -
                elements[0] * elements[7] * elements[13] -
                elements[4] * elements[1] * elements[15] +
                elements[4] * elements[3] * elements[13] +
                elements[12] * elements[1] * elements[7] -
                elements[12] * elements[3] * elements[5];

        temp[14] = -elements[0] * elements[5] * elements[14] +
                elements[0] * elements[6] * elements[13] +
                elements[4] * elements[1] * elements[14] -
                elements[4] * elements[2] * elements[13] -
                elements[12] * elements[1] * elements[6] +
                elements[12] * elements[2] * elements[5];

        temp[3] = -elements[1] * elements[6] * elements[11] +
                elements[1] * elements[7] * elements[10] +
                elements[5] * elements[2] * elements[11] -
                elements[5] * elements[3] * elements[10] -
                elements[9] * elements[2] * elements[7] +
                elements[9] * elements[3] * elements[6];

        temp[7] = elements[0] * elements[6] * elements[11] -
                elements[0] * elements[7] * elements[10] -
                elements[4] * elements[2] * elements[11] +
                elements[4] * elements[3] * elements[10] +
                elements[8] * elements[2] * elements[7] -
                elements[8] * elements[3] * elements[6];

        temp[11] = -elements[0] * elements[5] * elements[11] +
                elements[0] * elements[7] * elements[9] +
                elements[4] * elements[1] * elements[11] -
                elements[4] * elements[3] * elements[9] -
                elements[8] * elements[1] * elements[7] +
                elements[8] * elements[3] * elements[5];

        temp[15] = elements[0] * elements[5] * elements[10] -
                elements[0] * elements[6] * elements[9] -
                elements[4] * elements[1] * elements[10] +
                elements[4] * elements[2] * elements[9] +
                elements[8] * elements[1] * elements[6] -
                elements[8] * elements[2] * elements[5];

        double determinant = elements[0] * temp[0] + elements[1] * temp[4]
                + elements[2] * temp[8] + elements[3] * temp[12];

        determinant = 1.0 / determinant;

        Mat4 result = new Mat4();

        for (int i = 0; i < 4 * 4; i++)
            result.elements[i] = temp[i] * determinant;

        return result;
    }

    public Vec4 getColumn(int index)
    {
        return new Vec4(elements[index], elements[index + 4],
                elements[index + 2 * 4], elements[index + 3 * 4]);
    }

    public void setRow(int index, Vec4 row) {
        elements[index * 4] = row.x;
        elements[index * 4 + 1] = row.x;
        elements[index * 4 + 2] = row.x;
        elements[index * 4 + 3] = row.x;
    }

    public void setColumn(int index, Vec4 column)
    {
        elements[index] = column.x;
        elements[index + 4] = column.y;
        elements[index + 2 * 4] = column.z;
        elements[index + 3 * 4] = column.w;
    }

    public static Mat4 orthographic(double left, double right, double bottom, double top, double near, double far)
    {
        Mat4 result = new Mat4(1.0);

        result.elements[0] = 2.0 / (right - left);

        result.elements[1 + 4] = 2.0 / (top - bottom);

        result.elements[2 + 2 * 4] = 2.0 / (near - far);

        result.elements[3] = (left + right) / (left - right);
        result.elements[3 + 4] = (bottom + top) / (bottom - top);
        result.elements[3 + 2 * 4] = (far + near) / (far - near);

        return result;
    }

    public static Mat4 perspective(double fov, double aspectRatio, double near, double far)
    {
        Mat4 result = new Mat4(1.0);

        double q = 1.0 / StrictMath.tan(StrictMath.toRadians(0.5 * fov));
        double a = q / aspectRatio;

        double b = (near + far) / (near - far);
        double c = (2.0 * near * far) / (near - far);

        result.elements[0] = a;
        result.elements[1 + 4] = q;
        result.elements[2 + 2 * 4] = b;
        result.elements[2 + 3 * 4] = -1.0;
        result.elements[3 + 2 * 4] = c;

        return result;
    }

    public static Mat4 lookAt(Vec3 camera, Vec3 object, Vec3 up)
    {
        Mat4 result = identity();
        Vec3 f = (object.subtract(camera)).normalize();
        Vec3 s = f.cross(up.normalize());
        Vec3 u = s.cross(f);

        result.elements[0] = s.x;
        result.elements[4] = s.y;
        result.elements[2 * 4] = s.z;

        result.elements[1] = u.x;
        result.elements[1 + 4] = u.y;
        result.elements[1 + 2 * 4] = u.z;

        result.elements[2] = -f.x;
        result.elements[2 + 4] = -f.y;
        result.elements[2 + 2 * 4] = -f.z;

        return result.multiply(translate(new Vec3(-camera.x, -camera.y, -camera.z)));
    }

    public static Mat4 translate(Vec3 translation)
    {
        Mat4 result = new Mat4(1.0);

        result.elements[3] = translation.x;
        result.elements[3 + 4] = translation.y;
        result.elements[3 + 2 * 4] = translation.z;

        return result;
    }

    public static Mat4 rotate(double angle, Vec3 axis)
    {
        Mat4 result = new Mat4(1.0);

        double r = StrictMath.toRadians(angle);
        double c = StrictMath.cos(r);
        double s = StrictMath.sin(r);
        double omc = 1.0 - c;

        double x = axis.x;
        double y = axis.y;
        double z = axis.z;

        result.elements[0] = x * x * omc + c;
        result.elements[4] = y * x * omc + z * s;
        result.elements[2 * 4] = x * z * omc - y * s;

        result.elements[1] = x * y * omc - z * s;
        result.elements[1 + 4] = y * y * omc + c;
        result.elements[1 + 2 * 4] = y * z * omc + x * s;

        result.elements[2] = x * z * omc + y * s;
        result.elements[2 + 4] = y * z * omc - x * s;
        result.elements[2 + 2 * 4] = z * z * omc + c;

        return result;
    }

    public static Mat4 rotate(Quaternion quat)
    {
        Mat4 result = identity();

        double qx, qy, qz, qw, qx2, qy2, qz2, qxqx2, qyqy2, qzqz2, qxqy2, qyqz2, qzqw2, qxqz2, qyqw2, qxqw2;
        qx = quat.x;
        qy = quat.y;
        qz = quat.z;
        qw = quat.w;
        qx2 = (qx + qx);
        qy2 = (qy + qy);
        qz2 = (qz + qz);
        qxqx2 = (qx * qx2);
        qxqy2 = (qx * qy2);
        qxqz2 = (qx * qz2);
        qxqw2 = (qw * qx2);
        qyqy2 = (qy * qy2);
        qyqz2 = (qy * qz2);
        qyqw2 = (qw * qy2);
        qzqz2 = (qz * qz2);
        qzqw2 = (qw * qz2);

        result.setRow(0, new Vec4(((1.0 - qyqy2) - qzqz2), (qxqy2 - qzqw2), (qxqz2 + qyqw2), 0.0));
        result.setRow(1, new Vec4((qxqy2 + qzqw2), ((1.0 - qxqx2) - qzqz2), (qyqz2 - qxqw2), 0.0));
        result.setRow(2, new Vec4((qxqz2 - qyqw2), (qyqz2 + qxqw2), ((1.0 - qxqx2) - qyqy2), 0.0));
        return result;
    }

    public static Mat4 scale(Vec3 scale)
    {
        Mat4 result = new Mat4(1.0);

        result.elements[0] = scale.x;
        result.elements[1 + 4] = scale.y;
        result.elements[2 + 2 * 4] = scale.z;

        return result;
    }

    public static Mat4 transpose(Mat4 matrix)
    {
        return new Mat4(
                new Vec4(matrix.getRows()[0].x, matrix.getRows()[1].x, matrix.getRows()[2].x, matrix.getRows()[3].x),
                new Vec4(matrix.getRows()[0].y, matrix.getRows()[1].y, matrix.getRows()[2].y, matrix.getRows()[3].y),
                new Vec4(matrix.getRows()[0].z, matrix.getRows()[1].z, matrix.getRows()[2].z, matrix.getRows()[3].z),
                new Vec4(matrix.getRows()[0].w, matrix.getRows()[1].w, matrix.getRows()[2].w, matrix.getRows()[3].w)
        );
    }


    public Vec4[] getRows() {
        Vec4[] rows = new Vec4[4];

        for (int y = 0; y < 4; y++) {
            rows[y] = new Vec4(elements[y], elements[y + 1], elements[y + 2], elements[y + 3]);
        }

        return rows;
    }

    private void clear() {
        for (int i = 0; i < 16; i++) {
            elements[i] = 0;
        }
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Mat4)) return false;
        Mat4 mat4 = (Mat4) object;
        return Arrays.equals(elements, mat4.elements);
    }

    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("mat4: ");

        for (int i = 0; i < 4; i++) {
            result.append("(").append(getRows()[i].x).append(", ").append(getRows()[i].y).append(", ")
                    .append(getRows()[i].z).append(", ").append(getRows()[i].w).append(")");

            if (i < 3) {
                result.append(", ");
            }
        }

        return result.toString();
    }

}
