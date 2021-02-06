package frc.robot.StateMachine;


public class StateMachine {

    private IState currentState;

    public StateMachine(IState initialState) {
        currentState = initialState;
    }

    public void run() {

        IState nextState = currentState.execute();
        if(nextState != currentState){

            currentState.finish();
            currentState = nextState;
            currentState.initialize();
        }
    }
    public enum States() {

    }
}

