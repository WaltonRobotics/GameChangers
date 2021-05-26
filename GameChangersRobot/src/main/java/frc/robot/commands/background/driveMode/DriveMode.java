package frc.robot.commands.background.driveMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.Constants.SmartDashboardKeys.kNormalScaleFactorKey;
import static frc.robot.Constants.SmartDashboardKeys.kTurboScaleFactorKey;
import static frc.robot.OI.*;

public abstract class DriveMode {

    public abstract void feed();

    protected double getScaleFactor() {
        return (sTurboButton.get() || sSecondaryTurboButton.get() || sTertiaryTurboButton.get())
                ? SmartDashboard.getNumber(kTurboScaleFactorKey, kTurboScaleFactor)
                : SmartDashboard.getNumber(kNormalScaleFactorKey, kNormalScaleFactor);
    }

    public double getLeftJoystickY() {
        double rawValue = sLeftJoystick.getY();
        double scaleFactor = getScaleFactor();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        return -rawValue * scaleFactor;
    }

    public double getRightJoystickY() {
        double rawValue = sRightJoystick.getY();
        double scaleFactor = getScaleFactor();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        return -rawValue * scaleFactor;
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
        double rawValue = sRightJoystick.getX();
        double scaleFactor = getScaleFactor();

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        return rawValue * scaleFactor;
    }

}
