package frc.robot.stateMachine;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import static frc.robot.utils.UtilMethods.joinStrings;

public class StateMachine {

    private final String mName;
    private IState mCurrentState;

    public StateMachine(String name, IState initialState) {
        mName = name;

        if (initialState != null) {
            mCurrentState = initialState;
        } else {
            throw new IllegalArgumentException("Initial state must not be null!");
        }

        mCurrentState.initialize();
    }

    public void run() {
        if (mCurrentState.getName() == null) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    mCurrentState.getName() + " state has a null name!");
        } else {
            SmartDashboard.putString(joinStrings(" ", mName, "Current State"), mCurrentState.getName());
        }

        IState nextState = mCurrentState.execute();

        if (nextState != null) {
            if (!nextState.equals(mCurrentState)) {
                mCurrentState.finish();
                mCurrentState = nextState;
                mCurrentState.initialize();
            }
        } else {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "State machine \"" + mName
                    + "\" has effectively terminated due to a null state");
        }
    }

    public IState getCurrentState() {
        return mCurrentState;
    }

    public String getName() {
        return mName;
    }

}

