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

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

        public static final int kConveyorFrontSensorID = 4;
        public static final int kConveyorBackSensorID = 5;

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
        public static final double kIRSensorFlickeringTime = 0.1;
        public static final double kNudgeTime = 0.29;
        public static final double kNudgeVoltage = 8.0;
        public static final int kMaximumBallCapacity = (ContextFlags.kIsInfiniteRecharge ? 5 : 3);
        public static final int kFrontLoadingCapacity = 1;

    }

    public static class Shooter {

        public static final double kMeasurementTuningMinDutyCycle = 0.4;
        public static final double kMeasurementTuningMaxDutyCycle = 1.0;

    }

    public static class SmartDashboardKeys {

        public static final String kLeftVelocityPKey = "Drivetrain/Left Velocity P";
        public static final String kRightVelocityPKey = "Drivetrain/Right Velocity P";

        public static final String kShooterMeasurementPeriodKey = "Shooter/Measurement Period";
        public static final String kShooterMeasurementWindowKey = "Shooter/Measurement Window";

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