package frc.robot.auton;

public class AutonFlags {

    private static final AutonFlags sInstance = new AutonFlags();

    private boolean isInAuton = false;
    private boolean mAutonNeedsToIntake = false;

    public static AutonFlags getInstance() {
        return sInstance;
    }

    public boolean doesAutonNeedToIntake() {
        return mAutonNeedsToIntake;
    }

    public void setAutonNeedsToIntake(boolean autonNeedsToIntake) {
        this.mAutonNeedsToIntake = autonNeedsToIntake;
    }

    public boolean isInAuton() {
        return isInAuton;
    }

    public void setInAuton(boolean inAuton) {
        isInAuton = inAuton;
    }

}
