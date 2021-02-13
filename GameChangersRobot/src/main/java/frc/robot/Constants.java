package frc.robot;

public final class Constants {

    public static class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static class DriverPreferences {

        public static final double kDriveDeadband = 0.05;

    }

    public static class CANBusIDs {

        public static final int kLeftMaster = 3;
        public static final int kLeftSlave = 4;

        public static final int kRightMaster = 1;
        public static final int kRightSlave = 2;

    }

    public static class DioIDs {

        public static final int kRobotId1 = 8;
        public static final int kRobotId2 = 9;

    }

    public static class PIDSlots {

        public static final int kDrivetrainVoltageSlot = 0;
        public static final int kDrivetrainVelocitySlot = 1;

    }

    public static class SmartDashboardKeys {

        public static final String kLeftVelocityPKey = "Drivetrain/Left Velocity P";
        public static final String kRightVelocityPKey = "Drivetrain/Right Velocity P";

    }

    public static class LiveDashboardKeys {

        public static final String kLiveDashboardTableName = "Live Dashboard";
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
