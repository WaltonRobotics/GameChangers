package frc.robot.subsystems;

/*
    This class is intended to provide an interface for setting and querying
    flags for subsystems that default commands technically do not "require".
    For example, the LED controller default command might need to know
    whether the shooter is ready and the turret is aligned. These flags
    can then be set within the shooter and turret default commands
    respectively. This prevents unnecessary requirement conflicts between
    default commands.
 */
public class SubsystemFlags {

    private static final SubsystemFlags sInstance = new SubsystemFlags();

    private boolean mIsIntaking = false;
    private boolean mIsOuttaking = false;
    private boolean mIsShooting = false;
    private boolean mIsReadyToShoot = false;
    private boolean mIsTurretZeroingDisabled = false;
    private boolean mHasTurretZeroed = false;
    private boolean mIsClimberDeployed = false;
    private boolean mDoesTurretNeedToHomeForClimbing = false;
    private boolean mIsTurretHomedForClimbing = false;

    public static SubsystemFlags getInstance() {
        return sInstance;
    }

    public void resetAllStaticFlags() {
        setIsIntaking(false);
        setIsOuttaking(false);
        setIsShooting(false);
        setIsReadyToShoot(false);
        setIsTurretZeroingDisabled(false);
        setTurretHasZeroed(false);
        setIsClimberDeployed(false);
        setDoesTurretNeedToHomeForClimbing(false);
        setIsTurretHomedForClimbing(false);
    }

    public boolean isIntaking() {
        return mIsIntaking;
    }

    public void setIsIntaking(boolean isIntaking) {
        this.mIsIntaking = isIntaking;
    }

    public boolean isOuttaking() {
        return mIsOuttaking;
    }

    public void setIsOuttaking(boolean isOuttaking) {
        this.mIsOuttaking = isOuttaking;
    }

    public boolean isReadyToShoot() {
        return mIsReadyToShoot;
    }

    public void setIsReadyToShoot(boolean isReadyToShoot) {
        this.mIsReadyToShoot = isReadyToShoot;
    }

    public boolean isShooting() {
        return mIsShooting;
    }

    public void setIsShooting(boolean isShooting) {
        this.mIsShooting = isShooting;
    }

    public void setIsTurretZeroingDisabled(boolean isTurretZeroingDisabled) {
        this.mIsTurretZeroingDisabled = isTurretZeroingDisabled;
    }

    public boolean isTurretZeroingDisabled() {
        return mIsTurretZeroingDisabled;
    }

    public void setTurretHasZeroed(boolean hasTurretZeroed) {
        this.mHasTurretZeroed = hasTurretZeroed;
    }

    public boolean hasTurretZeroed() {
        return mHasTurretZeroed;
    }

    public boolean isClimberDeployed() {
        return mIsClimberDeployed;
    }

    public void setIsClimberDeployed(boolean isClimberDeployed) {
        this.mIsClimberDeployed = isClimberDeployed;
    }

    public boolean doesTurretNeedToHomeForClimbing() {
        return mDoesTurretNeedToHomeForClimbing;
    }

    public void setDoesTurretNeedToHomeForClimbing(boolean doesTurretNeedToHomeForClimbing) {
        this.mDoesTurretNeedToHomeForClimbing = doesTurretNeedToHomeForClimbing;
    }

    public boolean isTurretHomedForClimbing() {
        return mIsTurretHomedForClimbing;
    }

    public void setIsTurretHomedForClimbing(boolean isTurretHomedForClimbing) {
        this.mIsTurretHomedForClimbing = isTurretHomedForClimbing;
    }

}
