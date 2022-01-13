package frc.robot.commands.background.driveMode;

import static frc.robot.OI.sDriveGamepad;

public abstract class DriveMode {

    public abstract void feed();

    public double applyDeadband(double rawValue, double deadband) {
        if (Math.abs(rawValue) < deadband) {
            return 0;
        }

        return rawValue;
    }

    public double getLeftJoystickY() {
        return -sDriveGamepad.getLeftY();
    }

    public double getRightJoystickY() {
        return -sDriveGamepad.getRawAxis(3);
    }

    /* The following methods are for drive modes with separated turn and throttle commands (i.e. Curvature/Arcade). */

    /**
     * The left joystick is used for throttle.
     */
    public double getThrottle() {
        return getLeftJoystickY();
    }

    /**
     * The right joystick is used for turning.
     */
    public double getTurn() {
        return sDriveGamepad.getRightX();
    }

}
