package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriverPreferences.kDriveDeadband;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
    }

    @Override
    public void execute() {
        sDrivetrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());
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
