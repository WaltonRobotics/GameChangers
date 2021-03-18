package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

public class PracticeDeepSpace implements WaltRobot {
    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return null;
    }

    @Override
    public PIDController getDrivetrainLeftVoltagePID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainRightVoltagePID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainLeftVelocityPID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainRightVelocityPID() {
        return null;
    }

    @Override
    public ProfiledPIDController getDrivetrainTurnPID() {
        return null;
    }

    @Override
    public ProfiledPIDController getDrivetrainDriveStraightPowerPID() {
        return null;
    }

    @Override
    public ProfiledPIDController getDrivetrainDriveStraightHeadingPID() {
        return null;
    }

    @Override
    public double getDrivetrainPositionFactor() {
        return 0;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return 0;
    }

    @Override
    public double getDrivetrainTrackWidth() {
        return 0;
    }

    @Override
    public double getDrivetrainMaxVelocity() {
        return 0;
    }

    @Override
    public double getDrivetrainMaxAcceleration() {
        return 0;
    }

    @Override
    public double getLimelightMountingHeight() {
        return 0;
    }

    @Override
    public double getLimelightMountingAngle() {
        return 0;
    }

    @Override
    public void populateShooterInterpolationMethods() {

    }

    @Override
    public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap() {
        return null;
    }

    @Override
    public PolynomialRegression getShooterPolynomial() {
        return null;
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
