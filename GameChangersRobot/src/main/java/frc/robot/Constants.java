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


    }

    public class SmartDashboardKeys{

        public static final String kdriveStraightheadingPkey = "Drive Straight Heading P";
        public static final String kforwardP = "Forward P";
        public static final String kturnPkey = "Turn P";
        public static final String kleftEncodervalue = "Left Encoder Value";
        public static final String krightEncoderValue = "Right Encoder Value";
    }
}
