package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Robot.sDrivetrain;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad sGamePad = new Gamepad(kGamepadPort);

    public static EnhancedJoystickButton sResetDrivetrainButton = new EnhancedJoystickButton(sRightJoystick, 2);

    public static EnhancedJoystickButton shootButton = new EnhancedJoystickButton(sGamePad, 1);
    public static EnhancedJoystickButton retractButton = new EnhancedJoystickButton(sGamePad, 2);
    public static EnhancedJoystickButton deployButton = new EnhancedJoystickButton(sGamePad, 3);
    public static EnhancedJoystickButton intakingButton = new EnhancedJoystickButton(sGamePad, 4);

    static {
        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
    }

}
