package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.Constants.InputDevices.kLeftJoystickPort;
import static frc.robot.Constants.InputDevices.kRightJoystickPort;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);


}
