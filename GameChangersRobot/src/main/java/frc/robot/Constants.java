package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Constants{

    public class Inputs {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;

        public static final int SHIFT_UP_PORT = 3;
        public static final int SHIFT_DOWN_PORT = 2;
    }

    public static class Drivetrain{
        public static final int kRightMaster = 1;
        public static final int kRightSlave = 2;
        public static final int kLeftMaster = 3;
        public static final int kLeftSlave = 4;

    }

    public static class Shooter{
        public static final int kFlyMaster = 9;
        public static final int kFlySlave = 10;
    }

    public static class Intake{
        public static final int kIntakeMotor = 5;
        public static final int kIntakeToggle = 1;
    }
}
