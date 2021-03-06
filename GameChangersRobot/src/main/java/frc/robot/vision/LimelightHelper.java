package frc.robot.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.movingAverage.SimpleMovingAverage;

import static frc.robot.Constants.FieldConstants.kTargetHeightInches;
import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.Robot.sCurrentRobot;

public class LimelightHelper {

    private static boolean mIsLEDOn = true;

    private static final NetworkTable mTable = NetworkTableInstance.getDefault().getTable("limelight");

    private static final NetworkTableEntry mTx = mTable.getEntry("tx");
    private static final NetworkTableEntry mTy = mTable.getEntry("ty");
    private static final NetworkTableEntry mTa = mTable.getEntry("ta");
    private static final NetworkTableEntry mTv = mTable.getEntry("tv");
    private static final NetworkTableEntry mCamtran = mTable.getEntry("camtran");
    private static final NetworkTableEntry mLedMode = mTable.getEntry("ledMode");
    private static final NetworkTableEntry mCamMode = mTable.getEntry("camMode");
    private static final NetworkTableEntry mPipeline = mTable.getEntry("pipeline");

    private static final SimpleMovingAverage mTxMovingAverage = new SimpleMovingAverage(5);
    private static final SimpleMovingAverage mTyMovingAverage = new SimpleMovingAverage(5);

    private LimelightHelper() {
        // Update moving averages when tx and ty change
        // Only average in values when we see the target
//        mTx.addListener(event -> {
//            if (getTV() > 0) {
//                mTxMovingAverage.addData(event.value.getDouble());
//            }
//        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
//
//        mTy.addListener(event -> {
//            if (getTV() > 0) {
//                mTyMovingAverage.addData(event.value.getDouble());
//            }
//        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public static void updateData() {
        if (getTV() > 0) {
            mTxMovingAverage.addData(mTx.getDouble(0.0));
            mTyMovingAverage.addData(mTy.getDouble(0.0));
        }
    }

    /**
     * @return tx The x angle from target in degrees
     */
    public static double getTX() {
        return mTxMovingAverage.getMean();
    }

    /**
     * @return ty The y angle from target in degrees
     */
    public static double getTY() {
        return mTyMovingAverage.getMean();
    }

    /**
     * @return ta The area of the target
     */
    public static double getTA() {
        return mTa.getDouble(0);
    }

    /**
     * @return tv The number of targets in the field of view
     */
    public static double getTV() {
        return mTv.getDouble(0);
    }

    public static double[] getCamtran() {
        return mCamtran.getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    public static int getLEDMode() {
        return mLedMode.getNumber(0).intValue();
    }

    public static void setLEDMode(boolean on) {
        if (on) {
            mLedMode.setNumber(kLEDsOn);
        } else {
            mLedMode.setNumber(kLEDsOff);
        }
    }

    public static int getCamMode() {
        return mCamMode.getNumber(0).intValue();
    }

    public static void setCamMode(boolean isVisionProcessing) {
        if (isVisionProcessing) {
            mCamMode.setNumber(kVisionMode);
        } else {
            mCamMode.setNumber(kDriverMode);
        }
    }

    public static int getPipeline() {
        return mPipeline.getNumber(0).intValue();
    }

    public static void setPipeline(int pipelineNumber) {
        mPipeline.setNumber(pipelineNumber);
    }

    public static void toggleLimelight() {
        mIsLEDOn = !mIsLEDOn;
        setLEDMode(mIsLEDOn);
    }

    /**
     * @return distance The distance to the target in meters
     */
    public static double getDistanceToTargetMeters() {
        return Units.feetToMeters(getDistanceToTargetFeet());
    }

    public static double getDistanceToTargetFeet() {
        return ((kTargetHeightInches - sCurrentRobot.getCurrentRobot().getLimelightMountingHeight()) /
                (Math.tan(Units.degreesToRadians(sCurrentRobot.getCurrentRobot().getLimelightMountingAngle() + getTY()))))
                / 12;
    }

}
