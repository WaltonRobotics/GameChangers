package frc.robot;

public final class Constants {

    public static class ContextFlags {

        public static boolean kIsInCompetition = false;
        public static boolean kIsInTuningMode = true;
        public static boolean kIsInfiniteRecharge = false;

    }

    public static class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static class DriverPreferences {

        public static final double kDriveDeadband = 0.05;

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

    }

    public static class DioIDs {

        public static final int kConveyorFrontSensorID = 4;
        public static final int kConveyorBackSensorID = 5;

        public static final int kPixyCamReadLineID = 6;
        public static final int kLEDStripWriteLineID = 7;

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

        // The time that the IRSensor flickers randomly after changing states
        public static final double kIRSensorFlickeringTime = 0.75;
        public static final double kNudgeTime = 0.29;
        public static final double kNudgeVoltage = 8.0;
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
        public static final double kSpinDownTime = 0.25;

        public static final int kFlywheelEncoderPPR = 2048;
        public static final double kFlywheelDiameter = 4.0;

        public static final double kDefaultShootingDistanceFeet = 11.2;
        public static final double kDefaultVelocityRawUnits = 13000;

        // Change to false to use polynomial interpolation instead
        public static final boolean kUseInterpolationMap = true;

        public static final double kOptimalShootingDistance = 11;
        public static final double kOptimalShootingDistanceFloor = 10.2;
        public static final double kOptimalShootingDistanceCeiling = 12.0;

        public static final double kAbsoluteShootingDistanceFloor = 8.8;
        public static final double kAbsoluteShootingDistanceCeiling = 23.2;

    }

    public static final class Tuning {

        // The range of output duty cycles the shooter will typically be operating within
        // Used to tune the measurement period and window
        public static final double kShooterMeasurementTuningMinDutyCycle = 0.4;
        public static final double kShooterMeasurementTuningMaxDutyCycle = 1.0;

        public static final int kDrivetrainAccelerationWindow = 8;

    }

    public static class ProMini {

        // Tolerance of duty cycles read from DIO pins
        public static final double kDutyCycleTolerance = 0.1;

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

        public static final String kLeftVelocityPKey = "Drivetrain/Left Velocity P";
        public static final String kRightVelocityPKey = "Drivetrain/Right Velocity P";

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