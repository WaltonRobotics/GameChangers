package frc.robot;

import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import org.junit.Assert;
import org.junit.Test;

import java.util.Scanner;

public class StateMachineTest {

    private final IState mIdle;
    private final IState mTurningLightBulbOn;

    public StateMachineTest() {
        mIdle = new IState() {
            @Override
            public void initialize() {
                System.out.println("In the Idle state");
            }

            @Override
            public IState execute() {
                return mTurningLightBulbOn;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Idle";
            }
        };

        mTurningLightBulbOn = new IState() {
            @Override
            public void initialize() {
                System.out.println("Turned the light bulb on!");
            }

            @Override
            public IState execute() {
                return null;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Turning Light Bulb On";
            }
        };
    }

    @Test
    public void checkStateMachine() {
        StateMachine stateMachine = new StateMachine("Test State Machine", mIdle);

        stateMachine.run();

        Assert.assertEquals(stateMachine.getCurrentState(), mTurningLightBulbOn);
    }

}
