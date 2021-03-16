package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.AutoAlign;
import frc.robot.vision.LimelightHelper;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.kDriveDeadband;
import static frc.robot.OI.*;
import static frc.robot.Robot.driveModeChooser;
import static frc.robot.Robot.sDrivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        if (kIsInTuningMode) {
            sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
        }

        sAutoAlignButton.whenPressed(new AutoAlign().withTimeout(1.5));
    }

    @Override
    public void execute() {
//        sDrivetrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());

        driveModeChooser.getSelected().feed();
    }

    private double getLeftJoystickY() {
        if (Math.abs(sLeftJoystick.getY()) < kDriveDeadband)
            return 0;
        return -sLeftJoystick.getY();
    }

    private double getRightJoystickY() {
        if (Math.abs(sRightJoystick.getY()) < kDriveDeadband)
            return 0;
        return -sRightJoystick.getY();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
