package frc.robot.StateMachine;

public interface IState {

    public void initialize();
    public IState execute();
    public void finish();
}
