package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Constants {

    public class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public class Hardware {

        public static final int kLeftMaster = 3;
        public static final int kLeftSlave = 4;

        public static final int kRightMaster = 1;
        public static final int kRightSlave = 2;

        public static final int kRobotId1 = 8;
        public static final int kRobotId2 = 9;
    }

    public static class DrivetrainPIDSlots {

        public static final int VOLTAGE_PID_SLOT = 0;
        public static final int VELOCITY_PID_SLOT = 1;

    }

    public class SmartDashboardKeys {

        public static final String kdriveStraightheadingPkey = "Drive Straight Heading P";
        public static final String kturnPkey = "Turn P";
    }
}
