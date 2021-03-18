package frc.robot;

public final class Constants {

    public static class ContextFlags {

        public static boolean kIsInCompetition = false;
        public static boolean kIsInTuningMode = false;
        public static boolean kIsInfiniteRecharge = false;

    }

    public static class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static class DriverPreferences {

        public static final double kDriveJoystickDeadband = 0.1;
        public static final boolean kUseSquareCurve = true;

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

        public static final int kLEDStripWriteLineID = 6;
        public static final int kPixyCamReadLineID = 7;

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static class PneumaticsIDs {

        public static final int kIntakeSolenoidID = 1;

    }

    public static class PIDSlots {

        public static final int kDrivetrainVoltageSlot = 0;
        public static final int kDrivetrainVelocitySlot = 1;

        public static final int kShooterSpinningUpSlot = 0;
        public static final int kShooterShootingSlot = 1;

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

        // Change to false to use polynomial interpolation instead
        public static final boolean kUseInterpolationMap = true;

        public static final double kOptimalShootingDistanceFloorFeet = 10;
        public static final double kOptimalShootingDistanceCeilingFeet = 12;

        public static final double kAbsoluteShootingDistanceFloorFeet = 8.61;
        public static final double kAbsoluteShootingDistanceCeilingFeet = 22.38;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;

    }

    public static final class Tuning {

        // The range of output duty cycles the shooter will typically be operating within
        // Used to tune the measurement period and window
        public static final double kShooterMeasurementTuningMinDutyCycle = 0.4;
        public static final double kShooterMeasurementTuningMaxDutyCycle = 1.0;

        public static final int kDrivetrainAccelerationWindow = 8;

    }

    public static class ProMicro {

        public static final int kSerialPortBaudRate = 9600;
        public static final double kUpdateRateSeconds = 0.2;

    }

    public static class LimelightConstants {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

    }

    public static class FieldConstants {

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

        public static final String kAutoAlignHeadingSetpointKey = "Auto Align/Heading Setpoint Deg";
        public static final String kAutoAlignPositionErrorKey = "Auto Align/Position Error Deg";
        public static final String kAutoAlignVelocityErrorKey = "Auto Align/Velocity Error Deg/s";
        public static final String kAutoAlignTurnRateKey = "Auto Align/Turn Rate";
        public static final String kAutoAlignTurnPKey = "Auto Align/Turn P";
        public static final String kAutoAlignTurnIKey = "Auto Align/Turn I";
        public static final String kAutoAlignTurnDKey = "Auto Align/Turn D";

        public static final String kShooterMeasurementPeriodKey = "Shooter/Measurement Period";
        public static final String kShooterMeasurementWindowKey = "Shooter/Measurement Window";
        public static final String kShooterErrorRawUnitsKey = "Shooter/Error Raw Units";
        public static final String kShooterErrorRPSKey = "Shooter/Error RPS";
        public static final String kShooterErrorInchesKey = "Shooter/Error Inches Per Sec";
        public static final String kShooterFlywheelVelocityKey = "Shooter/Flywheel Velocity Raw Units";
        public static final String kShooterTuningSetpointRawUnitsKey = "Shooter/Tuning Setpoint Raw Units";
        public static final String kShooterLimelightDistanceFeetKey = "Shooter/Limelight Distance Feet";

        public static final String kConveyorFrontSensorStateKey = "Conveyor/Front Sensor State";
        public static final String kConveyorBackSensorStateKey = "Conveyor/Back Sensor State";
        public static final String kConveyorBallCountKey = "Conveyor/Ball Count";

        public static final String kProMicroLEDWriteMessageKey = "Pro Micro/LED Write Message";
        public static final String kProMicroPixyCamReadMessageKey = "Pro Micro/PixyCam Read Message";

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