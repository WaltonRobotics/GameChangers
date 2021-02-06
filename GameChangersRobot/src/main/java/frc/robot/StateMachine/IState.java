package frc.robot.StateMachine;

public interface IState {

    public StateMachine.States initialize();
    public StateMachine.States execute();
    public void finish();
}
