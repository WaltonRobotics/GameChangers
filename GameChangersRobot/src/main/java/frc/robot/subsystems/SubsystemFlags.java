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

    private boolean mIntaking = false;
    private boolean mReadyToShoot = false;

    public static SubsystemFlags getInstance() {
        return sInstance;
    }

    public boolean isIntaking() {
        return mIntaking;
    }

    public void setIntaking(boolean intaking) {
        this.mIntaking = intaking;
    }

    public boolean isReadyToShoot() {
        return mReadyToShoot;
    }

    public void setReadyToShoot(boolean readyToShoot) {
        this.mReadyToShoot = readyToShoot;
    }

}
