package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

import static frc.robot.Constants.Turret.kPositionClosedLoopErrorToleranceDegrees;

public class CompGameChangers extends WaltRobot {

    private static final double[][] mShooterDistanceToVelocityTable = {
            {8.61, 13000},
            {10.94, 11400},
            {12.83, 11400},
            {15.735, 11250},
            {17.254, 11350},
            {19.22, 11750},
            {22.38, 12425},
    };

    public CompGameChangers() {
        super(mShooterDistanceToVelocityTable, 2);

        ProfiledPIDController drivetrainTurnProfiledPID = new ProfiledPIDController(
                0.013, 0, 0,
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

        mShooterConfig.kLimelightMountingHeight = 22.5;
        mShooterConfig.kLimelightMountingAngle = 30;

        mIntakeConfig.kIsIntakeControllerInverted = false;
        mIntakeConfig.kIntakeDutyCycle = 0.6;
        mIntakeConfig.kOuttakeDutyCycle = -0.4;
        mIntakeConfig.kSettleTime = 0.5;

        mConveyorConfig.kIsFrontConveyorControllerInverted = true;
        mConveyorConfig.kIsBackConveyorControllerInverted = true;
        mConveyorConfig.kIRSensorFlickeringTimeSeconds = 0.75;
        mConveyorConfig.kNudgeTimeSeconds = 0.06;
        mConveyorConfig.kFrontConveyorNudgeVoltage = 6.0;
        mConveyorConfig.kBackConveyorNudgeVoltage = 6.0;
        mConveyorConfig.kFrontConveyorFeedVoltage = 12.0;
        mConveyorConfig.kBackConveyorFeedVoltage = 12.0;
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
        mTurretConfig.kPositionalP = 7.0;
        mTurretConfig.kPositionalI = 0.002;
        mTurretConfig.kPositionalD = 0.0;
        mTurretConfig.kPositionalIZone = 100;
        mTurretConfig.kPositionalMaxIntegralAccumulator = 0;

        ProfiledPIDController turretClosedLoopAutoAlignProfiledPID = new ProfiledPIDController(
                0.2, 0, 0,
                new TrapezoidProfile.Constraints(120, 850)
        );
        turretClosedLoopAutoAlignProfiledPID.enableContinuousInput(-180.0, 180.0);
        turretClosedLoopAutoAlignProfiledPID.setTolerance(kPositionClosedLoopErrorToleranceDegrees);

        mTurretConfig.closedLoopAutoAlignProfiledPID = turretClosedLoopAutoAlignProfiledPID;
    }

}
