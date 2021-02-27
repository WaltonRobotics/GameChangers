package frc.robot.stateMachine;

public interface IState {

    void initialize();

    IState execute();

    void finish();

    String getName();

}
