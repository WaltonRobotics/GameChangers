package frc.robot.vision;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.UtilMethods;

import static frc.robot.Robot.sTurret;

public class PnPHelper {

    public static Pose2d getEstimatedPose() {
        PnPData data = LimelightHelper.getPnPData();

//        Quaternion quaternion = Quaternion.fromEulerAngles(
//                new Vec3(Math.toRadians(roll), Math.toRadians(yaw), Math.toRadians(pitch)));
//        double limelightMountingAngle = Math.toDegrees(quaternion.toEulerAngles(Quaternion.EulerSequence.XYZ).x);
//        double limelightHeading = Math.toDegrees(quaternion.toEulerAngles(Quaternion.EulerSequence.YXZ).x; // Might be y

//        double robotHeading = UtilMethods.restrictAngle(limelightHeading
//                - sTurret.getCurrentRobotRelativeHeading().getDegrees(), -180, 180);

//        SmartDashboard.putNumber("Yaw", yaw);

        double limelightXInches = 30 * 12 + data.getZInches();
        double limelightYInches = 7.5 * 12 - data.getXInches();

        double turretRadiusInches = 11.375 / 2.0 + 1.5;
        double turretAngleRadians = sTurret.getCurrentRobotRelativeHeading().getRadians();
        double dX = Math.cos(turretAngleRadians) * turretRadiusInches;
        double dY = Math.sin(turretAngleRadians) * turretRadiusInches;
        double robotX = limelightXInches - dX;
        double robotY = limelightYInches - dY;
        double turretRelativeHeading = data.getYawDegrees() - sTurret.getCurrentRobotRelativeHeading().getDegrees();
        double intakeRelativeHeading = 180.0 + turretRelativeHeading;

        double offsetX = 288.75 - 296.5872;
        double offsetY = 90.75 - 91.554;

        return new Pose2d(Units.feetToMeters((robotX + offsetX) / 12.0), Units.feetToMeters((robotY + offsetY) / 12.0),
                Rotation2d.fromDegrees(UtilMethods.restrictAngle(intakeRelativeHeading, -180.0, 180.0)));
    }

}
