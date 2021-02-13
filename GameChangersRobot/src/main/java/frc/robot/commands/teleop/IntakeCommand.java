package frc.robot.commands.teleop;

import frc.robot.stateMachine.IState;

public class IntakeCommand {

    private IState mIdle = new IState() {
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
