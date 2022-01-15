package frc.robot.robots;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class CompDeepSpace extends WaltRobot {

    private static final double[][] mShooterDistanceToVelocityTable = {
            {10.33, 10900},
            {19.7, 10910},
            {30.89, 10950},
    };

    public CompDeepSpace() {
        super(mShooterDistanceToVelocityTable, 2);

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

        mDrivetrainConfig.linearFeedforward = new SimpleMotorFeedforward(.216, 4.58, 0.322);
        mDrivetrainConfig.angularFeedforward = new SimpleMotorFeedforward(.449, 4.51, .22);
        mDrivetrainConfig.leftVoltagePID = new PIDController(1.0, 0, 0);
        mDrivetrainConfig.rightVoltagePID = new PIDController(1.0, 0, 0);
        mDrivetrainConfig.leftVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.rightVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.turnProfiledPID = drivetrainTurnProfiledPID;
        mDrivetrainConfig.driveStraightProfiledPowerPID = drivetrainDriveStraightPowerProfiledPID;
        mDrivetrainConfig.driveStraightProfiledHeadingPID = drivetrainDriveStraightHeadingProfiledPID;
        mDrivetrainConfig.kPositionFactor = 1.0 / 36.797;
        mDrivetrainConfig.kVelocityFactor = mDrivetrainConfig.kPositionFactor / 60.0;
        mDrivetrainConfig.kTrackWidthMeters = 0.8285293767383018;
        mDrivetrainConfig.kMaxVelocityMetersPerSecond = Units.feetToMeters(8.0);
        mDrivetrainConfig.kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(2.0);
        mDrivetrainConfig.kLeftMaxVoltage = 12.0;
        mDrivetrainConfig.kRightMaxVoltage = 12.0;
        mDrivetrainConfig.kOpenLoopRampRate = 0.0;

        mShooterConfig.kSpinningUpF = 0.05006525;
        mShooterConfig.kSpinningUpP = 0.36;
        mShooterConfig.kSpinningUpI = 0.0002;
        mShooterConfig.kSpinningUpD = 0;
        mShooterConfig.kSpinningUpIZone = 800;
        mShooterConfig.kSpinningUpMaxIntegralAccumulator = 0;

        mShooterConfig.kShootingF = 0.05006525;
        mShooterConfig.kShootingP = 0.365;
        mShooterConfig.kShootingI = 0.0002;
        mShooterConfig.kShootingD = 0;
        mShooterConfig.kShootingIZone = 800;
        mShooterConfig.kShootingMaxIntegralAccumulator = 0;

        mShooterConfig.kMaxVoltage = 11.0;

        mShooterConfig.kLimelightMountingHeightInches = 22.5;
        mShooterConfig.kLimelightMountingAngleDegrees = 30;

        mIntakeConfig.kIsIntakeControllerInverted = true;
        mIntakeConfig.kIntakeDutyCycle = 0.8;
        mIntakeConfig.kOuttakeDutyCycle = -0.4;
        mIntakeConfig.kSettleTime = 0.5;

        mConveyorConfig.kIsFrontConveyorControllerInverted = true;
        mConveyorConfig.kIsBackConveyorControllerInverted = true;
        mConveyorConfig.kIRSensorFlickeringTimeSeconds = 0.75;
        mConveyorConfig.kNudgeTimeSeconds = 0.1;
        mConveyorConfig.kNudgingDownTimeSeconds = 0.1;
        mConveyorConfig.kFrontConveyorNudgeDutyCycle = 6.0;
        mConveyorConfig.kBackConveyorNudgeDutyCycle = 6.0;
        mConveyorConfig.kFrontConveyorFeedDutyCycle = 12.0;
        mConveyorConfig.kBackConveyorFeedDutyCycle = 12.0;
        mConveyorConfig.kFrontConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kBackConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kFrontConveyorOuttakeDutyCycle = -1.0;
        mConveyorConfig.kBackConveyorOuttakeDutyCycle = -1.0;

        mTurretConfig.kLimitSwitchPosition = Rotation2d.fromDegrees(90);
        mTurretConfig.kForwardSoftLimitRawUnits = 0;
        mTurretConfig.kReverseSoftLimitRawUnits = -460;
        final double kGearRatio = 230.0 / 30.0;
        final double kTicksPerDriverRotation = 177;
        mTurretConfig.kTicksPerDegree = kTicksPerDriverRotation * kGearRatio / 360.0;
        mTurretConfig.kPositionalP = 4.0;
        mTurretConfig.kPositionalI = 0.002;
        mTurretConfig.kPositionalD = 0.0;
        mTurretConfig.kPositionalIZone = 100;
        mTurretConfig.kPositionalMaxIntegralAccumulator = 0;
    }

}