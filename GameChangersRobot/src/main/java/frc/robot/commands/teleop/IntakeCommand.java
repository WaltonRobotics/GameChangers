package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Intake;

import static edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod;
import static frc.robot.OI.retractButton;
import static frc.robot.OI.deployButton;
import static frc.robot.OI.intakingButton;
import static frc.robot.Robot.intake;
import static frc.robot.Robot.shooter;

public class IntakeCommand extends CommandBase {
    public IntakeCommand() {addRequirements(intake);}

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
            } else if (intake.isDeployed() && intakingButton.get()) {
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
            intake.setIntakeDeployed(false);
        }

        @Override
        public IState execute() {
            return mIdle;
        }

        @Override
        public void finish() {

        }
    };

    private IState mDeploy = new IState() {
        @Override
        public void initialize() {
            intake.setIntakeDeployed(true);
        }

        @Override
        public IState execute() {
            return mIdle;
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
            intake.setRollerDutyCycles(1.5);
            if (!intakingButton.get()) {
                return mIdle;
            } else {
                return mIntaking;
            }

        }

        @Override
        public void finish() {

        }
    };

}
