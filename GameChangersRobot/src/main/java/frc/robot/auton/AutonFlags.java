package frc.robot.auton;

public class AutonFlags {

    private static final AutonFlags sInstance = new AutonFlags();

    private boolean mInAuton = false;
    private boolean mAutonNeedsToIntake = false;

    public static AutonFlags getInstance() {
        return sInstance;
    }

    public boolean isInAuton() {
        return mInAuton;
    }

    public void setIsInAuton(boolean isInAuton) {
        this.mInAuton = isInAuton;
    }

    public boolean doesAutonNeedToIntake() {
        return mAutonNeedsToIntake;
    }

    public void setAutonNeedsToIntake(boolean autonNeedsToIntake) {
        this.mAutonNeedsToIntake = autonNeedsToIntake;
    }

}
