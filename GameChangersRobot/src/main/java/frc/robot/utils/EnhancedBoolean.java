package frc.robot.utils;

public class EnhancedBoolean {

    private boolean mCurrentState;
    private boolean mPreviousState;

    public EnhancedBoolean(boolean currentState) {
        this.mCurrentState = currentState;
        mPreviousState = currentState;
    }

    public EnhancedBoolean() {
        this(false);
    }

    public boolean get() {
        return mCurrentState;
    }

    public void set(boolean newState) {
        mPreviousState = mCurrentState;
        mCurrentState = newState;
    }

    public boolean isRisingEdge() {
        return mCurrentState && !mPreviousState;
    }

    public boolean isFallingEdge() {
        return !mCurrentState && mPreviousState;
    }

    public boolean hasChanged() {
        return mCurrentState != mPreviousState;
    }

    @Override
    public String toString() {
        return "EnhancedBoolean{" +
                "currentState=" + mCurrentState +
                ", previousState=" + mPreviousState +
                '}';
    }
}