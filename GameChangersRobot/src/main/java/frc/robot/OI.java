package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.Inputs.*;
import static frc.robot.Constants.Shooter.kFlyMaster;
import static frc.robot.Constants.Shooter.kFlySlave;
import static frc.robot.Constants.Inputs.GAMEPAD_PORT;

public class OI {

    public static Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
    public static Gamepad  mGamePad = new Gamepad(GAMEPAD_PORT);

    public static TalonFX mFlyWheelMaster = new TalonFX(kFlyMaster);
    public static TalonFX mFlyWheelSlave = new TalonFX(kFlySlave);

    public static JoystickButton shootButton = new JoystickButton(mGamePad,1);

}
