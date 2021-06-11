package frc.robot.commands.background.driveMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.Constants.SmartDashboardKeys.kCurvatureTurnSensitivityKey;
import static frc.robot.Constants.SmartDashboardKeys.kTurboScaleFactorKey;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;

public class CurvatureDrive extends DriveMode {

    private double quickStopAccumulator;

    @Override
    public double getTurn() {
        double rawValue = sRightJoystick.getX();
        double scaleFactor = (sTurboButton.get() || sSecondaryTurboButton.get())
                ? SmartDashboard.getNumber(kTurboScaleFactorKey, kTurboScaleFactor)
                : SmartDashboard.getNumber(kCurvatureTurnSensitivityKey, kNormalScaleFactor);

        if (Math.abs(rawValue) < kDriveJoystickDeadband)
            return 0;

        return rawValue * scaleFactor;
    }

    @Override
    public void feed() {
        double xSpeed = getThrottle();
        double zRotation = getTurn();

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        if (kUseSquareCurve) {
            xSpeed = Math.copySign(Math.pow(xSpeed, 2), xSpeed);
            zRotation = Math.copySign(Math.pow(zRotation, 2), zRotation);
        }

        double angularPower;
        boolean overPower;

        if (sQuickTurnButton.get()) {
            if (Math.abs(xSpeed) < kQuickStopThreshold) {
                quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator
                        + kQuickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - quickStopAccumulator;

            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        sDrivetrain.setDutyCycles(leftMotorOutput, rightMotorOutput);
    }

}