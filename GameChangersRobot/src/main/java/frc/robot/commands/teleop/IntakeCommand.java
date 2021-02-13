package frc.robot.commands.teleop;

import frc.robot.stateMachine.IState;
import static frc.robot.OI.retractButton;
import static frc.robot.OI.deployButton;
import static frc.robot.OI.intakingButton;
import static frc.robot.Robot.intake;

public class IntakeCommand {

    private IState mIdle = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            if (retractButton.isRisingEdge()) {
                return mRetract;
            } else if (deployButton.isRisingEdge()) {
                return mDeploy;
            } else if (intake.isDeployed() && intakingButton.isRisingEdge()) {
                return mIntaking;
            }
            return mIdle;
        }

        @Override
        public void finish() {

        }
    };

    private IState mRetract = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            return null;
        }

        @Override
        public void finish() {

        }
    };

    private IState mDeploy = new IState() {
        @Override
        public void initialize() {
            System.out.println("Hello World!");

        }

        @Override
        public IState execute() {
            return null;
        }

        @Override
        public void finish() {

        }
    };

    private IState mIntaking = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            return null;
        }

        @Override
        public void finish() {

        }
    };

}
