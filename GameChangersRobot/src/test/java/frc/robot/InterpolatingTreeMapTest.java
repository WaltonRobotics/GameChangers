package frc.robot;

import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import org.junit.Assert;
import org.junit.Test;

public class InterpolatingTreeMapTest {

    @Test
    public void checkInterpolatingTreeMap() {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> testMap = new InterpolatingTreeMap<>();

        double[][] xy = {
                { 1, 45000 },
                { 2, 50000 },
                { 3, 60000 },
                { 4, 80000 },
                { 5, 110000 },
                { 6, 150000 },
                { 7, 200000 },
                { 8, 300000 },
                { 9, 500000 },
                { 10, 1000000 },
        };

        for (double[] tuple : xy) {
            testMap.put(new InterpolatingDouble(tuple[0]), new InterpolatingDouble(tuple[1]));
        }

        Assert.assertEquals(testMap.getInterpolated(new InterpolatingDouble(1.5)),
                new InterpolatingDouble(47500.0));
    }

}
