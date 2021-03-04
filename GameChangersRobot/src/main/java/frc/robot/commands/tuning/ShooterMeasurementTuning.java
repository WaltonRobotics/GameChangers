package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UtilMethods;

import static frc.robot.Constants.Shooter.kMeasurementTuningMaxDutyCycle;
import static frc.robot.Constants.Shooter.kMeasurementTuningMinDutyCycle;
import static frc.robot.OI.sGamepad;
import static frc.robot.Robot.sShooter;

public class ShooterMeasurementTuning extends CommandBase {

    public ShooterMeasurementTuning() {
        addRequirements(sShooter);
    }

    @Override
    public void execute() {
        double dutyCycle = UtilMethods.scaleRange(
                sGamepad.getRightX(),
                -1, 1,
                kMeasurementTuningMinDutyCycle, kMeasurementTuningMaxDutyCycle
        );

        sShooter.setOpenLoopDutyCycle(dutyCycle);
    }

}
