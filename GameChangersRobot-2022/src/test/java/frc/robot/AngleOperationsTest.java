package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Assert;
import org.junit.Test;

public class AngleOperationsTest {

    // Verify that results of angle operations are restricted to [-180, 180]
    @Test
    public void checkAngleRange() {
        Rotation2d result = Rotation2d.fromDegrees(-135).minus(
                Rotation2d.fromDegrees(-45).plus(Rotation2d.fromDegrees(180))
        );

        Assert.assertEquals(result.getDegrees(), 90, 0.01);
    }

}
