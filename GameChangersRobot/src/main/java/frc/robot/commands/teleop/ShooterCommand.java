package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;

import static frc.robot.OI.shootButton;
import static frc.robot.Robot.sShooter;

public class ShooterCommand extends CommandBase {
    public ShooterCommand() {addRequirements(sShooter);}
    double targetVelocity = 12500;
    double tolerance = 100;

    IState mOff = new IState() {
        @Override
        public void initialize() {
            sShooter.setOpenLoopDutyCycles(0);
        }


        @Override
        public IState execute() {
            sShooter.setOpenLoopDutyCycles(0);

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
            sShooter.setProfileSlot(0);
        }

        @Override
        public IState execute() {
            sShooter.setClosedLoopVelocity(targetVelocity);
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
            sShooter.setClosedLoopVelocity(targetVelocity);
            if (Math.abs(sShooter.getVelocity() - targetVelocity) <= tolerance) {
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
        stateMachine.run();
    }
}
