package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriverPreferences.kDriveDeadband;
import static frc.robot.OI.sLeftJoystick;
import static frc.robot.OI.sRightJoystick;
import static frc.robot.Robot.sDrivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);
    }

    @Override
    public void execute() {
        sDrivetrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());
    }

    private double getLeftJoystickY() {
        if(Math.abs(sLeftJoystick.getY()) < kDriveDeadband)
            return 0;
        return -sLeftJoystick.getY();
    }

    private double getRightJoystickY() {
        if(Math.abs(sRightJoystick.getY()) < kDriveDeadband)
            return 0;
        return -sRightJoystick.getY();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
