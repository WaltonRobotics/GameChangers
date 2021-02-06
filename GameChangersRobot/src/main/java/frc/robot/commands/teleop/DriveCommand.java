package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.InputDevices.kDeadband;
import static frc.robot.OI.sLeftJoystick;
import static frc.robot.OI.sRightJoystick;
import static frc.robot.Robot.drivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());

    }

    private double getLeftJoystickY() {
        if(Math.abs(sLeftJoystick.getY()) < kDeadband)
            return 0;
        return -sLeftJoystick.getY();
    }

    private double getRightJoystickY() {
        if(Math.abs(sRightJoystick.getY()) < kDeadband)
            return 0;
        return -sRightJoystick.getY();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
