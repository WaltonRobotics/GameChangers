package frc.robot.stateMachine;


public class StateMachine {

    private static IState currentState;

    public StateMachine(IState initialState) {
        if (initialState != null) {
            currentState = initialState;
        } else {
            throw new IllegalArgumentException("Initial state must not be null!");
        }
    }

    public static void run() {
        IState nextState = currentState.execute();

        if(nextState != null) {
            if (nextState != currentState) {
                currentState.finish();
                currentState = nextState;
                currentState.initialize();
            }
        } else {
            // TODO: Log this!
        }
    }
}

