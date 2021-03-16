package frc.robot.commands.background.driveMode;

import static frc.robot.Constants.DriverPreferences.kDriveDeadband;
import static frc.robot.OI.sLeftJoystick;
import static frc.robot.OI.sRightJoystick;
import static frc.robot.Robot.responseFunctionChooser;

public abstract class DriveMode {

    public abstract void feed();

    /**
     * Note that by default the areas outside the deadband are not scaled from 0 to -1 / 1.
     */
    double applyDeadband(double value) {
        return Math.abs(value) > kDriveDeadband ? value : 0;
    }

    double applyResponseFunction(double value) {
        return responseFunctionChooser.getSelected().getOutput(value);
    }

    double getLeftJoystickY() {
        return -sLeftJoystick.getY();
    }

    double getRightJoystickY() {
        return -sRightJoystick.getY();
    }

    /* The following methods are for drive modes with separated turn and throttle commands (i.e. Curvature/Arcade). */

    /**
     * The left joystick is used for throttle.
     */
    double getThrottle() {
        return getLeftJoystickY();
    }

    /**
     * The right joystick is used for turning.
     */
    double getTurn() {
        return sRightJoystick.getX();
    }

}
