package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

import static frc.robot.Constants.Turret.kAlignedThresholdDegrees;
import static frc.robot.Constants.Turret.kPositionClosedLoopErrorToleranceDegrees;

public class CompGameChangers extends WaltRobot {

    private static final double[][] mShooterDistanceToVelocityTable = {
            { 9.26, 12250 },
            { 12.86, 11250 },
            { 16.04, 11000 },
            { 20.04, 9975 },
            { 24.03, 10900 }
    };

    public CompGameChangers() {
        super(mShooterDistanceToVelocityTable, 2);

        ProfiledPIDController drivetrainTurnProfiledPID = new ProfiledPIDController(
                0.015, 0, 0,
                new TrapezoidProfile.Constraints(400, 400)
        );
        drivetrainTurnProfiledPID.enableContinuousInput(-180.0, 180.0);
        drivetrainTurnProfiledPID.setTolerance(0.5, 1.0);

        ProfiledPIDController drivetrainDriveStraightPowerProfiledPID = new ProfiledPIDController(
                0.8, 0, 0,
                new TrapezoidProfile.Constraints(1.5, 1.5)
        );
        drivetrainDriveStraightPowerProfiledPID.setTolerance(0.09);

        ProfiledPIDController drivetrainDriveStraightHeadingProfiledPID = new ProfiledPIDController(
                0.035, 0.0002, 0,
                new TrapezoidProfile.Constraints(8, 85)
        );
        drivetrainDriveStraightHeadingProfiledPID.enableContinuousInput(-180.0, 180.0);
        drivetrainDriveStraightHeadingProfiledPID.setTolerance(0.5);

        mDrivetrainConfig.linearFeedforward = new SimpleMotorFeedforward(0.237, 2.17, 0.306);
        mDrivetrainConfig.angularFeedforward = new SimpleMotorFeedforward(1.0, 1.0, 1.0);
        mDrivetrainConfig.leftVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.rightVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.leftVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.rightVelocityPID = new PIDController(0.35, 0, 0);
        mDrivetrainConfig.turnProfiledPID = drivetrainTurnProfiledPID;
        mDrivetrainConfig.driveStraightProfiledPowerPID = drivetrainDriveStraightPowerProfiledPID;
        mDrivetrainConfig.driveStraightProfiledHeadingPID = drivetrainDriveStraightHeadingProfiledPID;
        mDrivetrainConfig.kPositionFactor = 1.0 / 17.011875;
        mDrivetrainConfig.kVelocityFactor = mDrivetrainConfig.kPositionFactor / 60.0;
        mDrivetrainConfig.kTrackWidthMeters = 0.706142170304554;
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

        mIntakeConfig.kIsIntakeControllerInverted = false;
        mIntakeConfig.kIntakeDutyCycle = 0.5;
        mIntakeConfig.kOuttakeDutyCycle = -0.4;
        mIntakeConfig.kSettleTime = 0.5;

        mConveyorConfig.kIsFrontConveyorControllerInverted = true;
        mConveyorConfig.kIsBackConveyorControllerInverted = true;
        mConveyorConfig.kIRSensorFlickeringTimeSeconds = 0.75;
        mConveyorConfig.kNudgeTimeSeconds = 0.058;
        mConveyorConfig.kFrontConveyorNudgeDutyCycle = 0.5;
        mConveyorConfig.kBackConveyorNudgeDutyCycle = 0.5;
        mConveyorConfig.kFrontConveyorFeedDutyCycle = 1.0;
        mConveyorConfig.kBackConveyorFeedDutyCycle = 1.0;
        mConveyorConfig.kFrontConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kBackConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kFrontConveyorOuttakeDutyCycle = -1.0;
        mConveyorConfig.kBackConveyorOuttakeDutyCycle = -1.0;
        mConveyorConfig.kFrontConveyorMaxVoltage = 12.0;
        mConveyorConfig.kBackConveyorMaxVoltage = 12.0;

        mTurretConfig.kLimitSwitchPosition = Rotation2d.fromDegrees(90);
        mTurretConfig.kForwardSoftLimitRawUnits = 0;
        mTurretConfig.kReverseSoftLimitRawUnits = -460;
        final double kGearRatio = 230.0 / 30.0;
        final double kTicksPerDriverRotation = 177;
        mTurretConfig.kTicksPerDegree = kTicksPerDriverRotation * kGearRatio / 360.0;
        mTurretConfig.kPositionalP = 7.0;
        mTurretConfig.kPositionalI = 0.002;
        mTurretConfig.kPositionalD = 0.0;
        mTurretConfig.kPositionalIZone = 100;
        mTurretConfig.kPositionalMaxIntegralAccumulator = 50;

        mTurretConfig.kMotionMagicF = 1023.0 * 0.4 / 50.0;
        mTurretConfig.kMotionMagicP = 7;
        mTurretConfig.kMotionMagicI = 0.002;
        mTurretConfig.kMotionMagicD = 0.0;
        mTurretConfig.kMotionMagicIZone = 100;
        mTurretConfig.kMotionMagicMaxIntegralAccumulator = 0;
        mTurretConfig.kAcceleration = 225.0;
        mTurretConfig.kCruiseVelocity = 50.0;
        mTurretConfig.kSCurveStrength = 3;

        ProfiledPIDController turretClosedLoopAutoAlignProfiledPID = new ProfiledPIDController(
                0.013, 0, 0,
                new TrapezoidProfile.Constraints(60, 360)
        );
        turretClosedLoopAutoAlignProfiledPID.setTolerance(kAlignedThresholdDegrees);
        turretClosedLoopAutoAlignProfiledPID.setGoal(0.0);

        mTurretConfig.closedLoopAutoAlignProfiledPID = turretClosedLoopAutoAlignProfiledPID;
    }

}
