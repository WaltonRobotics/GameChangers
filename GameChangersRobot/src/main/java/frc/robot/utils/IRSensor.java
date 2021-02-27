package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.RobotController.getFPGATime;
import static frc.robot.Constants.Conveyor.kIRSensorFlickeringTime;

public class IRSensor extends DigitalInput {

    private final EnhancedBoolean mCurrentState;
    private boolean mIgnoreState;
    private double mIgnoreStateStartTime;

    /**
     * Create an instance of a Digital Input class. Creates a digital input given a channel.
     *
     * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
     */
    public IRSensor(int channel) {
        super(channel);

        mCurrentState = new EnhancedBoolean();
        mIgnoreState = false;
        mIgnoreStateStartTime = -1;
    }

    public void update() {
        SmartDashboard.putBoolean("Ignore state", mIgnoreState);

        if (mIgnoreState && getFPGATime() - mIgnoreStateStartTime >= kIRSensorFlickeringTime) {
            mIgnoreState = false;
        }

        if (!mIgnoreState) {
            mCurrentState.set(!super.get());
        }

        if (mCurrentState.hasChanged()) {
            mIgnoreState = true;
            mIgnoreStateStartTime = getFPGATime();
        }
    }

    @Override
    public boolean get() {
        return mCurrentState.get();
    }
}
