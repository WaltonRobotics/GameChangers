package frc.robot.commands.background.driveMode;

import static frc.robot.Robot.sDrivetrain;

public class TankDrive extends DriveMode {

    @Override
    public void feed() {
        double leftOutput = applyResponseFunction(applyDeadband(getLeftJoystickY()));
        double rightOutput = applyResponseFunction(applyDeadband(getRightJoystickY()));

        sDrivetrain.setDutyCycles(leftOutput, rightOutput);
    }

}
