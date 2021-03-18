package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

public class CompGameChangers implements WaltRobot {

    @Override
    public void populateShooterInterpolationMethods() {

    }

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return null;
    }

    @Override
    public ShooterConfig getShooterConfig() {
        return null;
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        return null;
    }

    @Override
    public ConveyorConfig getConveyorConfig() {
        return null;
    }

    @Override
    public TurretConfig getTurretConfig() {
        return null;
    }
}
