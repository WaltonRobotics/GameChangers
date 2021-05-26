package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;

import java.util.logging.Level;

import static frc.robot.Constants.Tuning.kShooterMeasurementTuningMaxDutyCycle;
import static frc.robot.Constants.Tuning.kShooterMeasurementTuningMinDutyCycle;
import static frc.robot.OI.sGamepad;
import static frc.robot.Robot.sShooter;

/**
 * This command is intended to map the gamepad joystick to the shooter motor such that
 * manual control is possible to tune the measurement period and window.
 * <p>
 * Here are the recommended instructions from Phoenix Tuner:
 * The general recommended procedure is to first set these two parameters to the minimal
 * value of ‘1’ (Measure change in position per 1ms, and no rolling average). Then plot
 * the measured velocity while manually driving the Talon SRX(s) with a joystick/gamepad.
 * Sweep the motor output to cover the expected range that the sensor will be expected to cover.
 * Unless the sensor velocity is considerably fast (hundreds of sensor units per sampling period) the
 * measurement will be very coarse (visual stair-stepping as the motor output is increased).
 * Increase the sampling period until the measured velocity is sufficiently granular.
 * At this point the sensor velocity will have minimal stair-stepping (good) but will be quite noisy.
 * Increase the rolling average window until the velocity plot is sufficiently smooth, but still
 * responsive enough to meet the timing requirements of the mechanism.
 */
public class ShooterMeasurementTuning extends CommandBase {

    public ShooterMeasurementTuning() {
        addRequirements(sShooter);
    }

    @Override
    public void initialize() {
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Use right joystick X on gamepad to sweep the motor output.");
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Refer to Phoenix Tuner documentation for how to set measurement settings.");
    }

    @Override
    public void execute() {
        double dutyCycle = UtilMethods.scaleRange(
                sGamepad.getRightX(),
                -1, 1,
                kShooterMeasurementTuningMinDutyCycle, kShooterMeasurementTuningMaxDutyCycle
        );

        sShooter.setOpenLoopDutyCycle(dutyCycle);
    }

}
