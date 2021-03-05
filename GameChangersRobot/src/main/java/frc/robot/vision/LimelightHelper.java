package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.movingAverage.SimpleMovingAverage;

import static frc.robot.Constants.FieldConstants.kTargetHeightInches;
import static frc.robot.Robot.sCurrentRobot;

public class LimelightHelper {

    private static boolean limelightToggle = false;

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTableEntry tv = table.getEntry("tv");
    private static NetworkTableEntry ledMode = table.getEntry("ledMode");
    private static NetworkTableEntry pipeline = table.getEntry("pipeline");
    private static NetworkTableEntry camtran = table.getEntry("camtran");

    private static final SimpleMovingAverage mTYMovingAverage = new SimpleMovingAverage(5);

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
        mTYMovingAverage.addData(ty.getDouble(0));
        return mTYMovingAverage.getMean();
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
        return Units.feetToMeters(getDistanceFeet());
    }

    public static double getDistanceFeet() {
        return ((kTargetHeightInches - sCurrentRobot.getCurrentRobot().getLimelightMountingHeight()) /
                (Math.tan(Units.degreesToRadians(sCurrentRobot.getCurrentRobot().getLimelightMountingAngle() + getTY()))))
                / 12;
    }

    public static void setPipeline(int pipelineNumber) {
        pipeline.setNumber(pipelineNumber);
    }

}
