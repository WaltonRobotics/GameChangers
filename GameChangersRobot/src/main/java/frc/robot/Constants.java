package frc.robot;

public final class Constants {

    public static class ContextFlags {

        public static final int kTeamNumber = 2974;
        public static final boolean kIsInCompetition = false;
        public static final boolean kIsInTuningMode = false;
        public static final boolean kIsInfiniteRecharge = false;

    }

    public static class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static class DriverPreferences {

        public static final double kDriveJoystickDeadband = 0.1;
        public static final boolean kUseSquareCurve = true;
        public static final double kAutoAlignTimeout = 1.5;
        public static final double kTurretMasterOverrideDeadband = 0.1;
        public static final double kNormalScaleFactor = 0.82;
        public static final double kTurboScaleFactor = 1.0;
        public static final double kTurretScaleFactor = 0.4;
        public static final double kQuickStopThreshold = 0.3;
        public static final double kQuickStopAlpha = 0.05;

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

    }

    public static class DioIDs {

        public static final int kConveyorFrontSensorID = 4;
        public static final int kConveyorBackSensorID = 5;

        public static final int kTurretLimitSwitchID = 6;

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static class PneumaticsIDs {

        public static final int kDeployIntakeSolenoidID = 1;
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
        public static final int kFrontLoadingCapacity = 2;

    }

    public static class Shooter {

        // The tolerance to exit the spinning up state and enter the shooting state
        public static final double kSpinningUpToleranceRawUnits = 300;
        // The tolerance to maintain the shooting state
        public static final double kShootingToleranceRawUnits = 150;

        // Short period of time after the shoot button is released where the flywheels
        // continue rotating to ensure last few shots don't go amiss
        public static final double kSpinDownTimeSeconds = 0.25;

        public static final int kFlywheelEncoderPPR = 2048;
        public static final double kFlywheelDiameterInches = 4.0;

        public static final double kDefaultShootingDistanceFeet = 11.2;
        public static final double kDefaultVelocityRawUnits = 11500;
        public static final double kBarfVelocityRawUnits = 6000;

        // Change to false to use polynomial interpolation instead
        public static final boolean kUseInterpolationMap = true;

        public static final double kOptimalShootingDistanceFloorFeet = 10;
        public static final double kOptimalShootingDistanceCeilingFeet = 12;

        public static final double kAbsoluteShootingDistanceFloorFeet = 8.61;
        public static final double kAbsoluteShootingDistanceCeilingFeet = 22.38;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;

        // Limelight NT takes up to 100ms to update at most
        public static final double kLimelightLEDWaitTimeSeconds = 0.2;

    }

    public static class Turret {

        public static final double kClosedLoopErrorTolerance = 3.77 * 2.0 / 3.0;
        public static final int kWithinToleranceLoopsToSettle = 5;
        public static final double kZeroingDutyCycle = 0.5;
        public static final double kZeroingTimeout = 2.0;

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

        public static final double kAlignedTolerance = 1.5;

    }

    public static class Limelight {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

    }

    public static class Field {

        public static final double kTargetHeightInches = 89.75;
        public static final double kTargetFrontOffsetFeet = 0.16;

    }

    public static class SmartDashboardKeys {

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
        public static final String kShooterTuningSetpointRawUnitsKey = "Shooter/Tuning Setpoint Raw Units";
        public static final String kShooterLimelightDistanceFeetKey = "Shooter/Limelight Distance Feet";
        public static final String kShooterAdjustableHoodStateKey = "Shooter/Adjustable Hood State";

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

        public static final String kProMicroLEDWriteMessageKey = "Pro Micro/LED Write Message";
        public static final String kProMicroPixyCamReadMessageKey = "Pro Micro/PixyCam Read Message";

        public static final String kNormalScaleFactorKey = "Driver Preferences/Normal Scale Factor";
        public static final String kTurboScaleFactorKey = "Driver Preferences/Turbo Scale Factor";

    }

    public static class LiveDashboardKeys {

        public static final String kLiveDashboardTableName = "Live_Dashboard";
        public static final String kRobotXKey = "robotX";
        public static final String kRobotYKey = "robotY";
        public static final String kRobotHeadingKey = "robotHeading";
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