package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.auton.LiveDashboard;

public class LiveDashboardHelper {

    public static void putRobotData(Pose2d currentPose) {
        LiveDashboard.getInstance().setRobotX(Units.metersToFeet(currentPose.getTranslation().getX()));
        LiveDashboard.getInstance().setRobotY(Units.metersToFeet(currentPose.getTranslation().getY()));
        LiveDashboard.getInstance().setRobotHeading(currentPose.getRotation().getRadians());
    }

    public static void putTrajectoryData(Pose2d trajectoryPose) {
        LiveDashboard.getInstance().setPathX(Units.metersToFeet(trajectoryPose.getTranslation().getX()));
        LiveDashboard.getInstance().setPathY(Units.metersToFeet(trajectoryPose.getTranslation().getY()));
        LiveDashboard.getInstance().setPathHeading(trajectoryPose.getRotation().getRadians());
    }

}
