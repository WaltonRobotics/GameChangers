package frc.robot.auton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.RamseteDebuggingKeys.*;

public class RamseteDebuggingTable {

    private static final RamseteDebuggingTable sInstance = new RamseteDebuggingTable();
    private final NetworkTable mRamseteDebuggingtable = NetworkTableInstance.getDefault().getTable(kRamseteDebuggingTableName);

    public static RamseteDebuggingTable getInstance() {
        return sInstance;
    }

    public double getLeftReference() {
        return mRamseteDebuggingtable.getEntry(kLeftReferenceKey).getDouble(0.0);
    }

    public void setLeftReference(double leftReference) {
        mRamseteDebuggingtable.getEntry(kLeftReferenceKey).setDouble(leftReference);
    }

    public double getLeftMeasurement() {
        return mRamseteDebuggingtable.getEntry(kLeftMeasurementKey).getDouble(0.0);
    }

    public void setLeftMeasurement(double leftMeasurement) {
        mRamseteDebuggingtable.getEntry(kLeftMeasurementKey).setDouble(leftMeasurement);
    }

    public double getRightReference() {
        return mRamseteDebuggingtable.getEntry(kRightReferenceKey).getDouble(0.0);
    }

    public void setRightReference(double rightReference) {
        mRamseteDebuggingtable.getEntry(kRightReferenceKey).setDouble(rightReference);
    }

    public double getRightMeasurement() {
        return mRamseteDebuggingtable.getEntry(kRightMeasurementKey).getDouble(0.0);
    }

    public void setRightMeasurement(double rightMeasurement) {
        mRamseteDebuggingtable.getEntry(kRightMeasurementKey).setDouble(rightMeasurement);
    }

}
