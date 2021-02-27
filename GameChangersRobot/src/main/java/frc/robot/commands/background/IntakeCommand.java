package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;

import static frc.robot.OI.*;
import static frc.robot.Robot.sIntake;

public class IntakeCommand extends CommandBase {

    private final IState mIdle;
    private final IState mDeploy;
    private final IState mRetract;
    private final IState mIntaking;

    private final StateMachine mStateMachine;

    public IntakeCommand() {
        addRequirements(sIntake);

        mIdle = new IState() {
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

        mDeploy = new IState() {
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

        mRetract = new IState() {
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

        mIntaking = new IState() {
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

        mStateMachine = new StateMachine(mIdle);
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }
    
}
