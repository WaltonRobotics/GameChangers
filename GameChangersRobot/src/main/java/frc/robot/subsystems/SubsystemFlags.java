package frc.robot.subsystems;

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
