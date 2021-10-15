package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;
import static frc.robot.utils.EnhancedJoystickButton.*;
import static frc.robot.utils.Gamepad.Button.*;

public class OI {

    public static Joystick sLeftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick sRightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad sDriveGamepad = new Gamepad(1);
    public static Gamepad sManipulationGamepad = new Gamepad(2);

    public static EnhancedJoystickButton sAutoAlignButton = new EnhancedJoystickButton(sRightJoystick, 1);
    public static EnhancedJoystickButton sQuickTurnButton = new EnhancedJoystickButton(sRightJoystick, 2);
    public static EnhancedJoystickButton sAutoAssistDriveStraightButton = new EnhancedJoystickButton(sRightJoystick, 3);
    public static EnhancedJoystickButton sResetDrivetrainButton = new EnhancedJoystickButton(sRightJoystick, 4);
    public static EnhancedJoystickButton sSecondaryAutoAssistDriveStraightButton = new EnhancedJoystickButton(sRightJoystick, 5);
    public static EnhancedJoystickButton sTertiaryAutoAssistDriveStraightButton = new EnhancedJoystickButton(sRightJoystick, 6);

    public static EnhancedJoystickButton sTurboButton = new EnhancedJoystickButton(sLeftJoystick, 7);
    public static EnhancedJoystickButton sSecondaryTurboButton = new EnhancedJoystickButton(sLeftJoystick, 8);

    public static EnhancedJoystickButton sShootButton = new EnhancedJoystickButton(sManipulationGamepad, RIGHT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sBarfButton = new EnhancedJoystickButton(sManipulationGamepad, RIGHT_BUMPER.getIndex());
    public static EnhancedJoystickButton sToggleLimelightLEDsButton = new EnhancedJoystickButton(sManipulationGamepad, POV_W);
    public static EnhancedJoystickButton sToggleShooterAdjustableHoodButton = new EnhancedJoystickButton(sManipulationGamepad, TOP_BUTTON.getIndex());

    public static EnhancedJoystickButton sRetractIntakeButton = new EnhancedJoystickButton(sManipulationGamepad, POV_N);
    public static EnhancedJoystickButton sDeployIntakeButton = new EnhancedJoystickButton(sManipulationGamepad, POV_S);
    public static EnhancedJoystickButton sIntakeButton = new EnhancedJoystickButton(sManipulationGamepad, LEFT_TRIGGER.getIndex());
    public static EnhancedJoystickButton sOuttakeButton = new EnhancedJoystickButton(sManipulationGamepad, LEFT_BUMPER.getIndex());

    public static EnhancedJoystickButton sOverrideFrontConveyorButton = new EnhancedJoystickButton(sManipulationGamepad, BACK_BUTTON.getIndex());
    public static EnhancedJoystickButton sOverrideBackConveyorButton = new EnhancedJoystickButton(sManipulationGamepad, START_BUTTON.getIndex());
    public static EnhancedJoystickButton sResetBallCountButton = new EnhancedJoystickButton(sManipulationGamepad, POV_E);

    public static EnhancedJoystickButton sAlignTurretButton = new EnhancedJoystickButton(sManipulationGamepad, LEFT_BUTTON.getIndex());
    public static EnhancedJoystickButton sHomeTurretButton = new EnhancedJoystickButton(sManipulationGamepad, DOWN_BUTTON.getIndex());
    public static EnhancedJoystickButton sZeroTurretButton = new EnhancedJoystickButton(sManipulationGamepad, RIGHT_BUTTON.getIndex());

    public static EnhancedJoystickButton sToggleClimberDeployButton = new EnhancedJoystickButton(sLeftJoystick, 1);
    public static EnhancedJoystickButton sLockClimberButton = new EnhancedJoystickButton(sManipulationGamepad, LEFT_STICK_BUTTON.getIndex());

    // Shooting challenge buttons
    public static EnhancedJoystickButton sCalibratePoseButton = new EnhancedJoystickButton(sLeftJoystick, 5);
    public static EnhancedJoystickButton sGoToReintroductionZoneButton = new EnhancedJoystickButton(sLeftJoystick, 2);
    public static EnhancedJoystickButton sHomeInterstellarAccuracyButton = new EnhancedJoystickButton(sLeftJoystick, 3);
    public static EnhancedJoystickButton sHomePowerPortButton = new EnhancedJoystickButton(sLeftJoystick, 4);
    public static EnhancedJoystickButton sBackUpTwoFeetForShooterCalibrationButton = new EnhancedJoystickButton(sLeftJoystick, 6);
    public static EnhancedJoystickButton sRunInterstellarRoutineButton = new EnhancedJoystickButton(sRightJoystick, 8);

}
