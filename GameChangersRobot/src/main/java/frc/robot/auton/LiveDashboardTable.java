package frc.robot.auton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.LiveDashboardKeys.*;

public class LiveDashboardTable {

    private static final LiveDashboardTable sInstance = new LiveDashboardTable();
    private final NetworkTable mLiveDashboardTable = NetworkTableInstance.getDefault().getTable(kLiveDashboardTableName);

    public static LiveDashboardTable getInstance() {
        return sInstance;
    }

    public double getRobotX() {
        return mLiveDashboardTable.getEntry(kRobotXKey).getDouble(0.0);
    }

    public void setRobotX(double robotX) {
        mLiveDashboardTable.getEntry(kRobotXKey).setDouble(robotX);
    }

    public double getRobotY() {
        return mLiveDashboardTable.getEntry(kRobotYKey).getDouble(0.0);
    }

    public void setRobotY(double robotY) {
        mLiveDashboardTable.getEntry(kRobotYKey).setDouble(robotY);
    }

    public double getRobotHeading() {
        return mLiveDashboardTable.getEntry(kRobotHeadingKey).getDouble(0.0);
    }

    public void setRobotHeading(double robotHeading) {
        mLiveDashboardTable.getEntry(kRobotHeadingKey).setDouble(robotHeading);
    }

    public boolean isFollowingPath() {
        return mLiveDashboardTable.getEntry(kIsFollowingPathKey).getBoolean(false);
    }

    public void setFollowingPath(boolean followingPath) {
        mLiveDashboardTable.getEntry(kIsFollowingPathKey).setBoolean(followingPath);
    }

    public double getPathX() {
        return mLiveDashboardTable.getEntry(kPathXKey).getDouble(0.0);
    }

    public void setPathX(double pathX) {
        mLiveDashboardTable.getEntry(kPathXKey).setDouble(pathX);
    }

    public double getPathY() {
        return mLiveDashboardTable.getEntry(kPathYKey).getDouble(0.0);
    }

    public void setPathY(double pathY) {
        mLiveDashboardTable.getEntry(kPathYKey).setDouble(pathY);
    }

    public double getPathHeading() {
        return mLiveDashboardTable.getEntry(kPathHeadingKey).getDouble(0.0);
    }

    public void setPathHeading(double pathHeading) {
        mLiveDashboardTable.getEntry(kPathHeadingKey).setDouble(pathHeading);
    }

}
