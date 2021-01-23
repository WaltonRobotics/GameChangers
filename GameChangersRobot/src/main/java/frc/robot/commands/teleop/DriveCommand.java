package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.driveTrain;
import static frc.robot.OI.leftJoystick;
import static frc.robot.OI.rightJoystick;
import static frc.robot.Robot.driveTrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(driveTrain);
    }

    public double getLeftJoystickY() {
        return -leftJoystick.getY();
    }

    public double getRightJoystickY() {
        return -rightJoystick.getY();
    }

    @Override
    public void execute() {
        driveTrain.setDutyCycles(getLeftJoystickY(), getRightJoystickY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}