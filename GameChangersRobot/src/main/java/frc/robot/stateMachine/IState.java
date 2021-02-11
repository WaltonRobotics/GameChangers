package frc.robot.stateMachine;

public interface IState {

    public void initialize();
    public IState execute();
    public void finish();
}
