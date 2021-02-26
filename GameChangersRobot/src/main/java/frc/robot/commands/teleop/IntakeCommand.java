package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;

import static frc.robot.OI.retractButton;
import static frc.robot.OI.deployButton;
import static frc.robot.OI.intakingButton;
import static frc.robot.Robot.sIntake;

public class IntakeCommand extends CommandBase {

    public IntakeCommand() {addRequirements(sIntake);}

    private IState mIdle = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            sIntake.setRollerDutyCycles(0.0);

            if (retractButton.isRisingEdge()) {
                return mRetract;
            } else if (deployButton.isRisingEdge()) {
                return mDeploy;
            } else if (sIntake.isDeployed() && (intakingButton.get()
                    || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToIntake()))) {
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
            sIntake.setIntakeDeployed(false);
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
            sIntake.setIntakeDeployed(true);
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
            sIntake.setRollerDutyCycles(0.8);

            if (!(intakingButton.get()
                    || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToIntake()))) {
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
