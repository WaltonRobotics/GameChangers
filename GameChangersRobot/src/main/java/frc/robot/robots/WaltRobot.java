package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

/* Generic interface for all Walton robots and their characteristics. */
public interface WaltRobot {

    /* Turret/shooter methods */
    double getLimelightMountingHeight();
    double getLimelightMountingAngle();

    void populateShooterInterpolationMethods();
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap();
    PolynomialRegression getShooterPolynomial();

    DrivetrainConfig getDrivetrainConfig();
    ShooterConfig getShooterConfig();
    IntakeConfig getIntakeConfig();
    ConveyorConfig getConveyorConfig();
    TurretConfig getTurretConfig();

}
