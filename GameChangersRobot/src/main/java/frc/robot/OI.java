package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.kLeftJoystickPort;
import static frc.robot.Constants.InputDevices.kRightJoystickPort;
import static frc.robot.Constants.InputDevices.kGamepadPort;
import static frc.robot.Robot.sDrivetrain;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad mGamePad = new Gamepad(kGamepadPort);

    public static EnhancedJoystickButton sResetDrivetrainButton = new EnhancedJoystickButton(sRightJoystick, 2);

    public static EnhancedJoystickButton shootButton = new EnhancedJoystickButton(mGamePad,1);
    public static EnhancedJoystickButton retractButton = new EnhancedJoystickButton(mGamePad, 2);
    public static EnhancedJoystickButton deployButton = new EnhancedJoystickButton(mGamePad, 3);
    public static EnhancedJoystickButton intakingButton = new EnhancedJoystickButton(mGamePad, 4);

    static {
        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
    }

}
