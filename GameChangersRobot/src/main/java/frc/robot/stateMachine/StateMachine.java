package frc.robot.stateMachine;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    }

    public void run() {
        SmartDashboard.putString(joinStrings(" ", mName, "Current State"), mCurrentState.getName());

        IState nextState = mCurrentState.execute();

        if (nextState != null) {
            if (nextState != mCurrentState) {
                mCurrentState.finish();
                mCurrentState = nextState;
                mCurrentState.initialize();
            }
        } else {
            System.out.println("[ERROR]: State machine has effectively terminated due to a null state.");
        }
    }

    public IState getCurrentState() {
        return mCurrentState;
    }

    public String getName() {
        return mName;
    }

}

