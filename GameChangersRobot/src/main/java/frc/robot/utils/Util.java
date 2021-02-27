package frc.robot.utils;

import java.util.Arrays;
import java.util.List;

public class Util {

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, String... strings) {
        StringBuilder sb = new StringBuilder();

        List<String> stringList = Arrays.asList(strings);

        for (int i = 0; i < stringList.size(); ++i) {
            sb.append(stringList.get(i));
            if (i < stringList.size() - 1) {
                sb.append(delim);
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

}
