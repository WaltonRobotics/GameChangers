package frc.robot.vision;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.robots.RobotIdentifier;
import frc.robot.utils.MovingAverage;

import static frc.robot.Robot.sCurrentRobot;

public class LimelightHelper {


    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTableEntry tv = table.getEntry("tv");
    private static NetworkTableEntry ledMode = table.getEntry("ledMode");
    private static NetworkTableEntry pipeline = table.getEntry("pipeline");
    private static NetworkTableEntry camtran = table.getEntry("camtran");

    private static double targetHeight = 89.69;
    private static final double FEET_OFFSET = 0.16;

    private static MovingAverage linearFilter = new MovingAverage(5, 0);
    private static boolean limelightToggle = false;

    private boolean limelightHasValidTarget;

    private LimelightHelper() {
    }

    /**
     * @return tx The x angle from target in degrees
     */
    public static double getTX() {
        return tx.getDouble(0);
    }

    /**
     * @return ty The y angle from target in degrees
     */
    public static double getTY() {
        linearFilter.update(ty.getDouble(0));
        return linearFilter.getDoubleOutput();
    }

    /**
     * @return ta The area of the target
     */
    public static double getTA() {
        return ta.getDouble(0);
    }

    /**
     * @return tv The number of targets in the field of view
     */
    public static double getTV() {
        return tv.getDouble(0);
    }

    public static double[] getCamtran() {
        return camtran.getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    public static void setLedMode(boolean on) {

        if (on) {
            ledMode.setNumber(3);
        } else {
            ledMode.setNumber(1);
        }

    }

    public static void toggleLimelight() {
        setLedMode(limelightToggle);
        limelightToggle = !limelightToggle;
    }

    /**
     * @return distance The distance to the target in meters
     */
    public static double getDistanceMeters() {
        return Units.inchesToMeters(targetHeight - sCurrentRobot.getCurrentRobot().getLimelightMountingHeight() / Math.tan(Units.degreesToRadians(sCurrentRobot.getCurrentRobot().getLimelightMountingAngle() + getTY())));
    }

    public static double getDistanceFeet() {
        return ((targetHeight - sCurrentRobot.getCurrentRobot().getLimelightMountingHeight()) / (Math.tan(Units.degreesToRadians(sCurrentRobot.getCurrentRobot().getLimelightMountingAngle() + getTY())))) / 12 + FEET_OFFSET;
    }

    public static void setPipeline(int pipelineNumber) {
        pipeline.setNumber(pipelineNumber);
    }
}

