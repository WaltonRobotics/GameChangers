package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.utils.EnhancedJoystickButton.POV_N;
import static frc.robot.utils.EnhancedJoystickButton.POV_S;
import static frc.robot.utils.Gamepad.Button.*;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad sGamePad = new Gamepad(kGamepadPort);

    public static EnhancedJoystickButton sResetDrivetrainButton = new EnhancedJoystickButton(sRightJoystick, 2);

    public static EnhancedJoystickButton sShootButton = new EnhancedJoystickButton(sGamePad, RIGHT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sBarfButton = new EnhancedJoystickButton(sGamePad, RIGHT_BUMPER.getIndex());

    public static EnhancedJoystickButton sRetractIntakeButton = new EnhancedJoystickButton(sGamePad, POV_N);
    public static EnhancedJoystickButton sDeployIntakeButton = new EnhancedJoystickButton(sGamePad, POV_S);
    public static EnhancedJoystickButton sIntakeButton = new EnhancedJoystickButton(sGamePad, LEFT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sOuttakeButton = new EnhancedJoystickButton(sGamePad, LEFT_BUMPER.getIndex());

}
