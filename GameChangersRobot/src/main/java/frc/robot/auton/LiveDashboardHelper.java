package frc.robot.auton;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;

public class LiveDashboardHelper {

    public static void putRobotData(Pose2d currentPose) {
        LiveDashboardTable.getInstance().setRobotX(Units.metersToFeet(currentPose.getTranslation().getX()));
        LiveDashboardTable.getInstance().setRobotY(Units.metersToFeet(currentPose.getTranslation().getY()));
        LiveDashboardTable.getInstance().setRobotHeading(currentPose.getRotation().getRadians());
    }

    public static void putTrajectoryData(Pose2d trajectoryPose) {
        LiveDashboardTable.getInstance().setPathX(Units.metersToFeet(trajectoryPose.getTranslation().getX()));
        LiveDashboardTable.getInstance().setPathY(Units.metersToFeet(trajectoryPose.getTranslation().getY()));
        LiveDashboardTable.getInstance().setPathHeading(trajectoryPose.getRotation().getRadians());
    }

}
