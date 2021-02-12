package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;

import static frc.robot.OI.shootButton;
import static frc.robot.Robot.shooter;

public class ShooterCommand extends CommandBase {

    double targetVelocity = 1.5;
    double tolerance = 1;

    IState mOff = new IState() {
        @Override
        public void initialize() {
            shooter.setOpenLoopDutyCycles(0);
        }


        @Override
        public IState execute() {
            shooter.setOpenLoopDutyCycles(0);

            if(shootButton.get()) {
                return mSpinningUp;
            }

            return mOff;
        }

        @Override
        public void finish() {
        }
    };

    IState mSpinningUp = new IState() {
        @Override
        public void initialize() {
            shooter.setProfileSlot(0);
        }

        @Override
        public IState execute() {
            shooter.setClosedLoopVelocity(targetVelocity);
            return mSpinningUp;
        }

        @Override
        public void finish() {
        }
    };

    IState mShooting = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            shooter.setClosedLoopVelocity(targetVelocity);
            if (Math.abs(shooter.getAverageClosedLoopVelocity() - targetVelocity) <= tolerance) {
                if (!shootButton.get()) {
                    return mOff;
                }
                return mShooting;
            } else {
                return mSpinningUp;
            }
        }

        @Override
        public void finish() {

        }
    };

    StateMachine stateMachine = new StateMachine(mOff);

    @Override
    public void execute() {

    }
}