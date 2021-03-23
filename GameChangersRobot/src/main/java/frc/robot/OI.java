package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;
import static frc.robot.utils.EnhancedJoystickButton.*;
import static frc.robot.utils.Gamepad.Button.*;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad sGamepad = new Gamepad(kGamepadPort);

    public static EnhancedJoystickButton sResetBallCountButton = new EnhancedJoystickButton(sGamepad, POV_E);

    public static EnhancedJoystickButton sTurboButton = new EnhancedJoystickButton(sLeftJoystick, 1);
    public static EnhancedJoystickButton sSecondaryTurboButton = new EnhancedJoystickButton(sLeftJoystick, 7);
    public static EnhancedJoystickButton sTertiaryTurboButton = new EnhancedJoystickButton(sLeftJoystick, 8);
    public static EnhancedJoystickButton sAutoAlignButton = new EnhancedJoystickButton(sRightJoystick, 1);
    public static EnhancedJoystickButton sShooterAdjustableToggleHoodButton = new EnhancedJoystickButton(sGamepad, TOP_BUTTON.getIndex());

    public static EnhancedJoystickButton sShootButton = new EnhancedJoystickButton(sGamepad, RIGHT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sBarfButton = new EnhancedJoystickButton(sGamepad, RIGHT_BUMPER.getIndex());
    public static EnhancedJoystickButton sToggleLimelightLEDsButton = new EnhancedJoystickButton(sGamepad, POV_W);

    public static EnhancedJoystickButton sRetractIntakeButton = new EnhancedJoystickButton(sGamepad, POV_N);
    public static EnhancedJoystickButton sDeployIntakeButton = new EnhancedJoystickButton(sGamepad, POV_S);
    public static EnhancedJoystickButton sIntakeButton = new EnhancedJoystickButton(sGamepad, LEFT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sOuttakeButton = new EnhancedJoystickButton(sGamepad, LEFT_BUMPER.getIndex());

    public static EnhancedJoystickButton sOverrideFrontConveyorButton = new EnhancedJoystickButton(sGamepad, BACK_BUTTON.getIndex());
    public static EnhancedJoystickButton sOverrideBackConveyorButton = new EnhancedJoystickButton(sGamepad, START_BUTTON.getIndex());

    public static EnhancedJoystickButton sAlignTurretButton = new EnhancedJoystickButton(sGamepad, DOWN_BUTTON.getIndex());
    public static EnhancedJoystickButton sHomeTurretButton = new EnhancedJoystickButton(sGamepad, LEFT_BUTTON.getIndex());
    public static EnhancedJoystickButton sQuickTurnButton = new EnhancedJoystickButton(sRightJoystick, 2);

    // Tuning buttons
    public static EnhancedJoystickButton sResetDrivetrainButton = new EnhancedJoystickButton(sLeftJoystick, 3);
    public static EnhancedJoystickButton sShooterTuningBackOneFootButton = new EnhancedJoystickButton(sLeftJoystick, 2);


}
