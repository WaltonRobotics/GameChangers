package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;

import static frc.robot.OI.*;
import static frc.robot.Robot.sIntake;

public class IntakeCommand extends CommandBase {

    private final IState mRetract = new IState() {
        @Override
        public void initialize() {
            sIntake.setDeployed(false);
        }

        @Override
        public IState execute() {
            return mIdle;
        }

        @Override
        public void finish() {

        }
    };
    private final IState mDeploy = new IState() {
        @Override
        public void initialize() {
            sIntake.setDeployed(true);
        }

        @Override
        public IState execute() {
            return mIdle;
        }

        @Override
        public void finish() {

        }
    };
    private final IState mIntaking = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            sIntake.setRollerDutyCycles(0.8);

            if (!(sIntakeButton.get()
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
    private final IState mIdle = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            sIntake.setRollerDutyCycles(0.0);

            if (sRetractIntakeButton.isRisingEdge()) {
                return mRetract;
            } else if (sDeployIntakeButton.isRisingEdge()) {
                return mDeploy;
            } else if (sIntake.isDeployed() && (sIntakeButton.get()
                    || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToIntake()))) {
                return mIntaking;
            }
            return mIdle;
        }

        @Override
        public void finish() {

        }
    };

    public IntakeCommand() {
        addRequirements(sIntake);
    }

}
