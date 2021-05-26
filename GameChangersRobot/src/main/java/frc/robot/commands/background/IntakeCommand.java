package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.SmartDashboardKeys.kIntakeIntakingDutyCycleKey;
import static frc.robot.OI.*;
import static frc.robot.Robot.sIntake;

public class IntakeCommand extends CommandBase {

    private final IState mIdle;
    private final IState mDeploy;
    private final IState mRetract;
    private final IState mIntaking;
    private final IState mOuttaking;

    private final StateMachine mStateMachine;

    public IntakeCommand() {
        addRequirements(sIntake);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sIntake.setRollerDutyCycle(0.0);

                if (sRetractIntakeButton.isRisingEdge()) {
                    return mRetract;
                } else if (sDeployIntakeButton.isRisingEdge()) {
                    return mDeploy;
                } else if (sIntakeButton.get()
                        || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToIntake())) {
                    return mIntaking;
                } else if (sOuttakeButton.get()) {
                    return mOuttaking;
                }

                return this;
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
//            private double mStartTime;

            @Override
            public void initialize() {
                sIntake.setDeployed(true);
                sIntake.setRetracted(false);

//                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
//                if (getFPGATimestamp() - mStartTime > sIntake.getConfig().kSettleTime) {
//                    sIntake.setDeployed(false);
//                }
//
//                sIntake.setRetracted(false);

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
                sIntake.setDeployed(false);
                sIntake.setRetracted(true);
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
                SubsystemFlags.getInstance().setIsIntaking(true);
            }

            @Override
            public IState execute() {
                if (kIsInTuningMode) {
                    sIntake.setRollerDutyCycle(SmartDashboard.getNumber(kIntakeIntakingDutyCycleKey,
                            sIntake.getConfig().kIntakeDutyCycle));
                } else {
                    sIntake.setRollerDutyCycle(sIntake.getConfig().kIntakeDutyCycle);
                }

                if (!(sIntakeButton.get()
                        || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToIntake()))) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {
                SubsystemFlags.getInstance().setIsIntaking(false);
            }

            @Override
            public String getName() {
                return "Intaking";
            }
        };

        mOuttaking = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIsOuttaking(true);
            }

            @Override
            public IState execute() {
                sIntake.setRollerDutyCycle(sIntake.getConfig().kOuttakeDutyCycle);

                if (!sOuttakeButton.get()) {
                    return mIdle;
                }

                return mOuttaking;
            }

            @Override
            public void finish() {
                SubsystemFlags.getInstance().setIsOuttaking(false);
            }

            @Override
            public String getName() {
                return "Outtaking";
            }
        };

        mStateMachine = new StateMachine("Intake", mIdle);
    }

    @Override
    public void initialize() {
//        sIntake.setDeployed(false);
//        sIntake.setRetracted(true);
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
