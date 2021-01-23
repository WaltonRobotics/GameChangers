package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.Inputs.*;

public class OI {

    public static Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

    public static TalonFX mFlyWheelMaster = new TalonFX(9);
    public static TalonFX mFlyWheelSlave = new TalonFX(10);
}
