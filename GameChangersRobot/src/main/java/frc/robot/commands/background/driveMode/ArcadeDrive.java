package frc.robot.commands.background.driveMode;

import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.DriverPreferences.kUseSquareCurve;
import static frc.robot.Robot.sDrivetrain;

public class ArcadeDrive extends DriveMode {

    @Override
    public void feed() {
        double xSpeed = getThrottle();
        double zRotation = getTurn();

        if (kUseSquareCurve) {
            xSpeed = Math.copySign(Math.pow(xSpeed, 2), xSpeed);
            zRotation = Math.copySign(Math.pow(zRotation, 2), zRotation);
        }

        sDrivetrain.setArcadeSpeeds(xSpeed, zRotation);
    }

}
