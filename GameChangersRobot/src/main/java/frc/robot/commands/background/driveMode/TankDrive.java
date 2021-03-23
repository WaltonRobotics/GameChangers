package frc.robot.commands.background.driveMode;

import static frc.robot.Constants.DriverPreferences.kUseSquareCurve;
import static frc.robot.Robot.sDrivetrain;

public class TankDrive extends DriveMode {

    @Override
    public void feed() {
        double leftOutput = getLeftJoystickY();
        double rightOutput = getRightJoystickY();

        if (kUseSquareCurve) {
            leftOutput = Math.copySign(Math.pow(leftOutput, 2), leftOutput);
            rightOutput = Math.copySign(Math.pow(rightOutput, 2), rightOutput);
        }

        sDrivetrain.setDutyCycles(leftOutput, rightOutput);
    }

}
