package frc.robot.auton;

public class AutonFlags {

    private static final AutonFlags sInstance = new AutonFlags();

    private boolean mIsInAuton = false;
    private boolean mIsAutonTurretZeroingEnabled = false;
    private boolean mDoesAutonNeedToIntake = false;
    private boolean mDoesAutonNeedToShoot = false;
    private boolean mDoesAutonNeedToAlignTurretFieldRelative = false;
    private boolean mDoesAutonNeedToAlignTurret = false;

    public static AutonFlags getInstance() {
        return sInstance;
    }

    public void resetAllStaticFlags() {
        setIsInAuton(false);
        setIsAutonTurretZeroingEnabled(false);
        setDoesAutonNeedToIntake(false);
        setDoesAutonNeedToShoot(false);
        setDoesAutonNeedToAlignTurretFieldRelative(false);
        setDoesAutonNeedToAlignTurret(false);
    }

    public boolean isInAuton() {
        return mIsInAuton;
    }

    public void setIsInAuton(boolean isInAuton) {
        this.mIsInAuton = isInAuton;
    }

    public void setIsAutonTurretZeroingEnabled(boolean isAutonTurretZeroingEnabled) {
        this.mIsAutonTurretZeroingEnabled = isAutonTurretZeroingEnabled;
    }

    public boolean isAutonTurretZeroingEnabled() {
        return mIsAutonTurretZeroingEnabled;
    }

    public boolean doesAutonNeedToIntake() {
        return mDoesAutonNeedToIntake;
    }

    public void setDoesAutonNeedToIntake(boolean doesAutonNeedToIntake) {
        this.mDoesAutonNeedToIntake = doesAutonNeedToIntake;
    }

    public void setDoesAutonNeedToShoot(boolean doesAutonNeedToShoot) {
        this.mDoesAutonNeedToShoot = doesAutonNeedToShoot;
    }

    public boolean doesAutonNeedToShoot() {
        return mDoesAutonNeedToShoot;
    }

    public boolean doesAutonNeedToAlignTurretFieldRelative() {
        return mDoesAutonNeedToAlignTurretFieldRelative;
    }

    public void setDoesAutonNeedToAlignTurretFieldRelative(boolean doesAutonNeedToAlignTurretFieldRelative) {
        this.mDoesAutonNeedToAlignTurretFieldRelative = doesAutonNeedToAlignTurretFieldRelative;
    }

    public void setDoesAutonNeedToAlignTurret(boolean doesAutonNeedToAlignTurret) {
        this.mDoesAutonNeedToAlignTurret = doesAutonNeedToAlignTurret;
    }

    public boolean doesAutonNeedToAlignTurret() {
        return mDoesAutonNeedToAlignTurret;
    }

}
