package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.AutoAlign;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        if (kIsInTuningMode) {
            sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
        }

        sAutoAlignButton.whenPressed(new AutoAlign().withTimeout(kAutoAlignTimeout));
    }

    @Override
    public void execute() {
        sDrivetrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());
    }

    private double getLeftJoystickY() {
        double rawValue =  sLeftJoystick.getY();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        if (kUseSquareCurve) {
            return -Math.copySign(Math.pow(rawValue, 2), rawValue);
        }

        return -rawValue;
    }

    private double getRightJoystickY() {
        double rawValue =  sRightJoystick.getY();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        if (kUseSquareCurve) {
            return -Math.copySign(Math.pow(rawValue, 2), rawValue);
        }

        return -rawValue;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
