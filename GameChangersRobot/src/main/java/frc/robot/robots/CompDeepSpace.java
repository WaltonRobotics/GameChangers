package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

import static frc.robot.Constants.Turret.kPositionClosedLoopErrorToleranceDegrees;

public class CompDeepSpace implements WaltRobot {

    private final DrivetrainConfig mDrivetrainConfig;

    public CompDeepSpace() {
        populateShooterInterpolationMethods();

        ProfiledPIDController drivetrainTurnProfiledPID = new ProfiledPIDController(
                0.05, 0, 0,
                new TrapezoidProfile.Constraints(360, 80)
        );
        drivetrainTurnProfiledPID.enableContinuousInput(-180.0, 180.0);
        drivetrainTurnProfiledPID.setTolerance(0.5, 1.0);

        ProfiledPIDController drivetrainDriveStraightPowerProfiledPID = new ProfiledPIDController(
                0.8, 0, 0,
                new TrapezoidProfile.Constraints(1.5, 1.5)
        );
        drivetrainDriveStraightPowerProfiledPID.setTolerance(0.09);

        ProfiledPIDController drivetrainDriveStraightHeadingProfiledPID = new ProfiledPIDController(
                0.2, 0, 0,
                new TrapezoidProfile.Constraints(60, 30)
        );
        drivetrainDriveStraightHeadingProfiledPID.enableContinuousInput(-180.0, 180.0);
        drivetrainDriveStraightHeadingProfiledPID.setTolerance(0.5);

        mDrivetrainConfig = new DrivetrainConfig();
        mDrivetrainConfig.linearFeedforward = new SimpleMotorFeedforward(0.191, 4.56, 0.0295);
        mDrivetrainConfig.angularFeedforward = new SimpleMotorFeedforward(0.409, 4.74, -0.11);
        mDrivetrainConfig.leftVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.rightVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.leftVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.rightVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.turnProfiledPID = drivetrainTurnProfiledPID;
        mDrivetrainConfig.driveStraightProfiledPowerPID = drivetrainDriveStraightPowerProfiledPID;
        mDrivetrainConfig.driveStraightProfiledHeadingPID = drivetrainDriveStraightHeadingProfiledPID;
        mDrivetrainConfig.kPositionFactor = 1.0 / 36.797;
        mDrivetrainConfig.kVelocityFactor = mDrivetrainConfig.kPositionFactor / 60.0;
        mDrivetrainConfig.kTrackWidthMeters = 0.8206803070586121;
        mDrivetrainConfig.kMaxVelocityMetersPerSecond = Units.feetToMeters(8.0);
        mDrivetrainConfig.kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(2.0);
        mDrivetrainConfig.kLeftMaxVoltage = 12.0;
        mDrivetrainConfig.kRightMaxVoltage = 12.0;
        mDrivetrainConfig.kOpenLoopRampRate = 0.0;
    }

    @Override
    public void populateShooterInterpolationMethods() {

    }

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return mDrivetrainConfig;
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
