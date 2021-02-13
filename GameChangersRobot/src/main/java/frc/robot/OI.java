package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.InputDevices.kLeftJoystickPort;
import static frc.robot.Constants.InputDevices.kRightJoystickPort;
import static frc.robot.Robot.sDrivetrain;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);

    public static JoystickButton sResetDrivetrainButton = new JoystickButton(sRightJoystick, 2);

    static {
        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
    }

}
