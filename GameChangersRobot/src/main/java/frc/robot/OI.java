package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.EnhancedJoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;
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

    public static EnhancedJoystickButton shootButton = new EnhancedJoystickButton(mGamePad,1);
    public static EnhancedJoystickButton retractButton = new EnhancedJoystickButton(mGamePad, 2);
    public static EnhancedJoystickButton deployButton = new EnhancedJoystickButton(mGamePad, 3);
    public static EnhancedJoystickButton intakingButton = new EnhancedJoystickButton(mGamePad, 4);

    public static VictorSPX mFrontConveyorMotor = new VictorSPX(kFrontConveyorMotor);
    public static VictorSPX mBackConveyorMotor = new VictorSPX(kBackConveyorMotor);

}
