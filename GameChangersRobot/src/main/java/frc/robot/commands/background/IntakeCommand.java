package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

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
                SubsystemFlags.getInstance().setIntaking(false);
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

            @Override
            public String getName() {
                return "Idle";
            }
        };

        mDeploy = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIntaking(false);

                sIntake.setDeployed(true);
            }

            @Override
            public IState execute() {
                return mIdle;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Deploy";
            }
        };

        mRetract = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIntaking(false);

                sIntake.setDeployed(false);
            }

            @Override
            public IState execute() {
                return mIdle;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Retract";
            }
        };

        mIntaking = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIntaking(true);
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

            @Override
            public String getName() {
                return "Intaking";
            }
        };

        mStateMachine = new StateMachine("Intake", mIdle);
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }

}
