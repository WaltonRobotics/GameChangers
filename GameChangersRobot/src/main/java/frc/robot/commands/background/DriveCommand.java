package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.AutoAlign;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.Constants.SmartDashboardKeys.kNormalScaleFactorKey;
import static frc.robot.Constants.SmartDashboardKeys.kTurboScaleFactorKey;
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

    private double getScaleFactor() {
        return sTurboButton.get() ? SmartDashboard.getNumber(kTurboScaleFactorKey, kTurboScaleFactor)
                : SmartDashboard.getNumber(kNormalScaleFactorKey, kNormalScaleFactor);
    }

    private double getLeftJoystickY() {
        double rawValue =  sLeftJoystick.getY();
        double scaleFactor = getScaleFactor();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        if (kUseSquareCurve) {
            return -Math.copySign(Math.pow(rawValue, 2), rawValue) * scaleFactor;
        }

        return -rawValue * (sTurboButton.get() ? kTurboScaleFactor : kNormalScaleFactor) * scaleFactor;
    }

    private double getRightJoystickY() {
        double rawValue =  sRightJoystick.getY();
        double scaleFactor = getScaleFactor();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        if (kUseSquareCurve) {
            return -Math.copySign(Math.pow(rawValue, 2), rawValue) * scaleFactor;
        }

        return -rawValue * scaleFactor;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
