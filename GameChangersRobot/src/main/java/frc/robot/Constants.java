package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static class ContextFlags {

        public static final int kTeamNumber = 2974;
        public static final boolean kIsInCompetition = false;
        public static final boolean kIsInTuningMode = false;
        public static final boolean kIsInfiniteRecharge = true;

    }

    public static class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static class DriverPreferences {

        public static final double kDriveJoystickDeadband = 0.1;
        public static final boolean kUseSquareCurve = false;
        public static final double kAutoAlignTimeout = 1.5;
        public static final double kTurretMasterOverrideDeadband = 0.1;
        public static final double kNormalScaleFactor = 1.0;
        public static final double kTurboScaleFactor = 1.0;
        public static final double kTurretScaleFactor = 0.4;
        public static final double kQuickStopThreshold = 0.3;
        public static final double kQuickStopAlpha = 0.05;
        public static final double kAutoAssistDriveStraightThrottleLimit = 0.9;

    }

    public static class CANBusIDs {

        public static final int kDrivetrainLeftMasterID = 3;
        public static final int kDrivetrainLeftSlaveID = 4;
        public static final int kDrivetrainRightMasterID = 1;
        public static final int kDrivetrainRightSlaveID = 2;

        public static final int kIntakeID = 5;

        public static final int kFrontConveyorID = 7;
        public static final int kBackConveyorID = 8;

        public static final int kFlywheelMasterID = 9;
        public static final int kFlywheelSlaveID = 10;

        public static final int kTurretID = 11;

        public static final int kClimberID = 12;

    }

    public static class DioIDs {

        public static final int kConveyorFrontSensorID = 4;
        public static final int kConveyorBackSensorID = 5;

        public static final int kTurretLimitSwitchID = 6;

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static class PneumaticsIDs {

        public static final int kDrivetrainGearShiftSolenoidID = 0;
        public static final int kDeployIntakeSolenoidID = 1;
        public static final int kClimberLockSolenoidID = 2;
        public static final int kClimberDeploySolenoidID = 3;
        public static final int kRetractIntakeSolenoidID = 4;
        public static final int kAdjustableHoodSolenoidID = 5;

    }

    public static class PIDSlots {

        public static final int kDrivetrainVoltageSlot = 0;
        public static final int kDrivetrainVelocitySlot = 1;

        public static final int kShooterSpinningUpSlot = 0;
        public static final int kShooterShootingSlot = 1;

        public static final int kTurretMotionMagicSlot = 0;
        public static final int kTurretPositionalSlot = 1;

    }

    public static class Conveyor {

        public static final int kMaximumBallCapacity = (ContextFlags.kIsInfiniteRecharge ? 5 : 3);
        public static final int kFrontLoadingCapacity = (ContextFlags.kIsInfiniteRecharge ? 1 : 0);

    }

    public static class Shooter {

        // The tolerance to exit the spinning up state and enter the shooting state
        public static final double kSpinningUpToleranceRawUnits = 150;
        // The tolerance to maintain the shooting state
        public static final double kShootingToleranceRawUnits = 200;

        // Short period of time after the shoot button is released where the flywheels
        // continue rotating to ensure last few shots don't go amiss
        public static final double kSpinDownTimeSeconds = 0.25;

        public static final int kFlywheelEncoderPPR = 2048;
        public static final double kFlywheelDiameterInches = 4.0;

        public static final double kDefaultShootingDistanceFeet = 11.2;
        public static final double kDefaultVelocityRawUnits = 11500;
        public static final double kBarfVelocityRawUnits = 6000;

        // Change to false to use polynomial regression instead
        public static final boolean kUseInterpolationMap = true;

        public static final double kOptimalShootingDistanceFloorFeet = 10;
        public static final double kOptimalShootingDistanceCeilingFeet = 12;

        public static final double kAbsoluteShootingDistanceFloorFeet = 5.0;
        public static final double kAbsoluteShootingDistanceCeilingFeet = 37.0;

    }

    public static class Turret {

        // Talon SRX positional PID constants
        public static final double kPositionClosedLoopErrorToleranceDegrees = 2.0 / 3.0;
        public static final double kClimbingHomedToleranceDegrees = 5.0;
        public static final int kWithinToleranceLoopsToSettle = 5;

        // Zeroing
        public static final double kZeroingDutyCycle = 0.5;
        public static final double kZeroingTimeout = 1.0;

        // Homing
        public static final double kHomingTimeout = 1.25;

        // Closed-loop aiming with Limelight as the feedback device constants
        public static final double kAimingKp = 0.011;
        public static final double kMinimumAimThresholdDegrees = 1.0;
        public static final double kMinimumAimDutyCycle = 0.13;
        public static final double kAlignedThresholdDegrees = 0.5;
        public static final double kAlignmentTimeout = 2.0;
        public static final double kAlignmentFieldRelativeTimeout = 1.0;

    }

    public static final class Tuning {

        // The range of output duty cycles the shooter will typically be operating within
        // Used to tune the measurement period and window
        public static final double kShooterMeasurementTuningMinDutyCycle = 0.4;
        public static final double kShooterMeasurementTuningMaxDutyCycle = 1.0;

        public static final int kDrivetrainAccelerationWindow = 8;

        public static final double kDrivetrainTuningSettingsUpdateRateSeconds = 1.5;

    }

    public static class ProMicro {

        public static final int kSerialPortBaudRate = 9600;
        public static final double kUpdateRateSeconds = 0.2;

    }

    public static class Limelight {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;
        public static final int kCamtranWindowSize = 5;

        public static final int kAlignmentPipeline = 0;
        public static final int kPnPPipeline = 1;

        public static final double kMaximumLEDWaitTimeSeconds = 0.5;

    }

    public static class Field {

        public static final double kTargetHeightInches = 89.75;
        public static final Rotation2d kTargetFieldRelativeHeading = Rotation2d.fromDegrees(180);

        public static final double kFieldWidthFeet = 30.0;
        public static final double kFieldHeightFeet = 15.0;

        public static final double kBreakPlaneSafetyOffsetFeet = 0.5;
        public static final double kGalacticSearchBreakPlaneLineMeters = Units.feetToMeters(27.5
                + kBreakPlaneSafetyOffsetFeet);

        public static final double kEndOfZoneOneFromTargetFeet = 9.5;

        public static final double kMinimumInterstellarHomingRadiusMeters = Units.feetToMeters(1.5);
        public static final double kMaximumInterstellarHomingRadiusMeters = Units.feetToMeters(5.5);

        public static final Pose2d kInterstellarHomingPose
                = new Pose2d(Units.feetToMeters(24.0625), Units.feetToMeters(7.5), Rotation2d.fromDegrees(90));

        public static final Pose2d kPowerPortScoringZonePose
                = new Pose2d(Units.feetToMeters(3.5 + 110 / 12.0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180));

        public static final Pose2d kPowerPortReintroductionZonePose =
                new Pose2d(Units.feetToMeters(3.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180));

    }

    public static class SmartDashboardKeys {

        public static final String kRobotIdentifierKey = "Robot/Robot Identifier";

        public static final String kDrivetrainLeftVelocityPKey = "Drivetrain/Left Velocity P";
        public static final String kDrivetrainRightVelocityPKey = "Drivetrain/Right Velocity P";
        public static final String kDrivetrainAngularVelocityKey = "Drivetrain/Angular Velocity Deg/s";
        public static final String kDrivetrainHeadingKey = "Drivetrain/Heading (deg)";
        public static final String kDrivetrainLeftPositionKey = "Drivetrain/Left Encoder Position Meters";
        public static final String kDrivetrainRightPositionKey = "Drivetrain/Right Encoder Position Meters";
        public static final String kDrivetrainLeftVelocityKey = "Drivetrain/Left Encoder Velocity Meters/s";
        public static final String kDrivetrainRightVelocityKey = "Drivetrain/Right Encoder Velocity Meters/s";
        public static final String kDrivetrainTuningOpenLoopRampRateKey = "Drivetrain/Tuning Open Loop Ramp Rate";

        public static final String kTurnToAngleHeadingSetpointKey = "Turn to Angle/Heading Setpoint Deg";
        public static final String kTurnToAnglePositionErrorKey = "Turn to Angle/Position Error Deg";
        public static final String kTurnToAngleVelocityErrorKey = "Turn to Angle/Velocity Error Deg/s";
        public static final String kTurnToAngleRateKey = "Turn to Angle/Rate";
        public static final String kTurnToAnglePKey = "Turn to Angle/Turn P";
        public static final String kTurnToAngleIKey = "Turn to Angle/Turn I";
        public static final String kTurnToAngleDKey = "Turn to Angle/Turn D";

        public static final String kDriveStraightDistanceAverageKey = "Drive Straight/Distance Average";
        public static final String kDriveStraightForwardPKey = "Drive Straight/Forward P";
        public static final String kDriveStraightForwardIKey = "Drive Straight/Forward I";
        public static final String kDriveStraightForwardDKey = "Drive Straight/Forward D";
        public static final String kDriveStraightHeadingPKey = "Drive Straight/Heading P";
        public static final String kDriveStraightHeadingIKey = "Drive Straight/Heading I";
        public static final String kDriveStraightHeadingDKey = "Drive Straight/Heading D";
        public static final String kDriveStraightForwardErrorKey = "Drive Straight/Forward Error";
        public static final String kDriveStraightForwardRateKey = "Drive Straight/Forward Rate";
        public static final String kDriveStraightHeadingErrorKey = "Drive Straight/Heading Error";
        public static final String kDriveStraightTurnRateKey = "Drive Straight/Turn Rate";

        public static final String kShooterMeasurementPeriodKey = "Shooter/Measurement Period";
        public static final String kShooterMeasurementWindowKey = "Shooter/Measurement Window";
        public static final String kShooterErrorRawUnitsKey = "Shooter/Error Raw Units";
        public static final String kShooterErrorRPSKey = "Shooter/Error RPS";
        public static final String kShooterErrorInchesKey = "Shooter/Error Inches Per Sec";
        public static final String kShooterFlywheelVelocityKey = "Shooter/Flywheel Velocity Raw Units";
        public static final String kShooterCurrentSetpointRawUnitsKey = "Shooter/Current Setpoint Raw Units";
        public static final String kShooterTuningSetpointRawUnitsKey = "Shooter/Tuning Setpoint Raw Units";
        public static final String kShooterLimelightDistanceFeetKey = "Shooter/Limelight Distance Feet";
        public static final String kShooterIsAdjustableHoodUpKey = "Shooter/Is Adjustable Hood Up";

        public static final String kIntakeIntakingDutyCycleKey = "Intake/Intaking Duty Cycle";

        public static final String kConveyorFrontSensorStateKey = "Conveyor/Front Sensor State";
        public static final String kConveyorBackSensorStateKey = "Conveyor/Back Sensor State";
        public static final String kConveyorBallCountKey = "Conveyor/Ball Count";

        public static final String kTurretForwardLimitStateKey = "Turret/Forward Limit State";
        public static final String kTurretRobotRelativeHeadingRawUnitsKey = "Turret/Robot-relative Heading Raw Units";
        public static final String kTurretRobotRelativeHeadingDegreesKey = "Turret/Robot-relative Heading Degrees";
        public static final String kTurretFieldRelativeHeadingDegreesKey = "Turret/Field-relative Heading Degrees";
        public static final String kTurretAngularVelocityRawUnitsKey = "Turret/Angular Velocity Raw Units";
        public static final String kTurretControlStateKey = "Turret/Control State";
        public static final String kTurretSetpointKey = "Turret/Setpoint";
        public static final String kTurretClosedLoopErrorDegreesKey = "Turret/Closed Loop Error Degrees";
        public static final String kTurretIsHomedForClimbingKey = "Turret/Is Homed For Climbing";

        public static final String kClimberIsUnlockedKey = "Climber/Is Unlocked";
        public static final String kClimberIsDeployedKey = "Climber/Is Deployed";

        public static final String kProMicroLEDWriteMessageKey = "Pro Micro/LED Write Message";
        public static final String kProMicroPixyCamReadMessageKey = "Pro Micro/PixyCam Read Message";

        public static final String kNormalScaleFactorKey = "Driver Preferences/Normal Scale Factor";
        public static final String kTurboScaleFactorKey = "Driver Preferences/Turbo Scale Factor";
        public static final String kCurvatureTurnSensitivityKey = "Driver Preferences/Turn Sensitivity";

        public static final String kLimelightSolvePnPXInchesKey = "Limelight Solve PnP/X Inches";
        public static final String kLimelightSolvePnPYInchesKey = "Limelight Solve PnP/Y Inches";
        public static final String kLimelightSolvePnPZInchesKey = "Limelight Solve PnP/Z Inches";
        public static final String kLimelightSolvePnPPitchDegreesKey = "Limelight Solve PnP/Pitch Degrees";
        public static final String kLimelightSolvePnPYawDegreesKey = "Limelight Solve PnP/Yaw Degrees";
        public static final String kLimelightSolvePnPRollDegreesKey = "Limelight Solve PnP/Roll Degrees";

    }

    public static class LiveDashboardKeys {

        public static final String kLiveDashboardTableName = "Live_Dashboard";
        public static final String kRobotXKey = "robotX";
        public static final String kRobotYKey = "robotY";
        public static final String kRobotHeadingKey = "robotHeading";
        public static final String kTurretRobotRelativeHeadingKey = "turretRobotRelativeHeading";
        public static final String kIsFollowingPathKey = "isFollowingPath";
        public static final String kPathXKey = "pathX";
        public static final String kPathYKey = "pathY";
        public static final String kPathHeadingKey = "pathHeading";

    }

    public static class RamseteDebuggingKeys {

        public static final String kRamseteDebuggingTableName = "Ramsete Debugging";
        public static final String kLeftReferenceKey = "leftReference";
        public static final String kLeftMeasurementKey = "leftMeasurement";
        public static final String kRightReferenceKey = "rightReference";
        public static final String kRightMeasurementKey = "rightMeasurement";

    }

}