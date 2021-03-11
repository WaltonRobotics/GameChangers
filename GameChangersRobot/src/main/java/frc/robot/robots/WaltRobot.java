package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

/* Generic interface for all Walton robots and their characteristics. */
public interface WaltRobot {

    /* Drivetrain methods */
    SimpleMotorFeedforward getDrivetrainFeedforward();

    PIDController getDrivetrainLeftVoltagePID();

    PIDController getDrivetrainRightVoltagePID();

    PIDController getDrivetrainLeftVelocityPID();

    PIDController getDrivetrainRightVelocityPID();

    ProfiledPIDController getDrivetrainTurnPID();

    ProfiledPIDController getDrivetrainDriveStraightPowerPID();
    ProfiledPIDController getDrivetrainDriveStraightHeadingPID();

    double getDrivetrainPositionFactor();

    double getDrivetrainVelocityFactor();

    double getTrackWidth();

    double getLimelightMountingHeight();

    double getLimelightMountingAngle();

    void populateShooterInterpolationMethods();
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap();
    PolynomialRegression getShooterPolynomial();

}
