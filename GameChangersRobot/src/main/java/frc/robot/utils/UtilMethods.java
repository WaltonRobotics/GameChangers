package frc.robot.utils;

import java.util.Arrays;
import java.util.List;

public class UtilMethods {

    public static double limitMagnitude(double value, double maxMagnitude) {
        return limitRange(value, -maxMagnitude, maxMagnitude);
    }

    public static double limitRange(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    public static double scaleRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
        double oldRange = oldMax - oldMin;

        if (oldRange == 0) {
            return newMin;
        } else {
            double newRange = newMax - newMin;
            return (((value - oldMin) * newRange) / oldRange) + newMin;
        }
    }

    public static String joinStrings(String delimeter, String... strings) {
        StringBuilder sb = new StringBuilder();

        List<String> stringList = Arrays.asList(strings);

        for (int i = 0; i < stringList.size(); ++i) {
            sb.append(stringList.get(i));
            if (i < stringList.size() - 1) {
                sb.append(delimeter);
            }
        }
        return sb.toString();
    }

    public static boolean isWithinTolerance(double actualValue, double targetValue, double tolerance) {
        return Math.abs(actualValue - targetValue) <= tolerance;
    }

    public static boolean allWithinTolerance(List<Double> list, double value, double tolerance) {
        boolean result = true;
        for (Double value_in : list) {
            result &= isWithinTolerance(value_in, value, tolerance);
        }
        return result;
    }

    public static double restrictAngle(double value, double min, double max) {
        if (value == 180.0) return 180.0;
        if (value == -180.0) return -180.0;

        double width = max - min;
        double offsetValue = value - min;

        return (offsetValue - (Math.floor(offsetValue / width) * width)) + min;
    }

    public static double getDifferenceBetweenAngles(double source, double target) {
        double a = source - target;
        a += (a > 180) ? -360 : (a < -180) ? 360 : 0;
        return a;
    }

}
