package frc.robot.auton;

public class AutonFlags {

    private static final AutonFlags sInstance = new AutonFlags();

    private boolean mIsInAuton = false;
    private boolean mDoesAutonNeedToIntake = false;

    public static AutonFlags getInstance() {
        return sInstance;
    }

    public boolean isInAuton() {
        return mIsInAuton;
    }

    public void setIsInAuton(boolean isInAuton) {
        this.mIsInAuton = isInAuton;
    }

    public boolean doesAutonNeedToIntake() {
        return mDoesAutonNeedToIntake;
    }

    public void setDoesAutonNeedToIntake(boolean doesAutonNeedToIntake) {
        this.mDoesAutonNeedToIntake = doesAutonNeedToIntake;
    }

}