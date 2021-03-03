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
    private boolean mIsReadyToShoot = false;

    public static SubsystemFlags getInstance() {
        return sInstance;
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

}