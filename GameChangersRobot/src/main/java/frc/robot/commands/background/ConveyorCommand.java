package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.SmartDashboardKeys.kConveyorTuningNudgeTime;
import static frc.robot.OI.*;
import static frc.robot.Robot.sConveyor;

public class ConveyorCommand extends CommandBase {

    private final IState mIdle;
    private final IState mIntaking;
    private final IState mOuttaking;
    private final IState mNudging;
    private final IState mNudgingDown;
    private final IState mFeeding;

    private final StateMachine mStateMachine;

    private double nudgeStart;

    public ConveyorCommand() {
        addRequirements(sConveyor);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                if (sOverrideFrontConveyorButton.get()) {
                    sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorIntakeDutyCycle);
                } else {
                    sConveyor.setFrontDutyCycle(0.0);
                }

                if (sOverrideBackConveyorButton.get()) {
                    sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorIntakeDutyCycle);
                } else {
                    sConveyor.setBackDutyCycle(0.0);
                }

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Idle";
            }
        };

        mIntaking = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                if (sConveyor.getBallCount() < 4) {
                    sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorIntakeDutyCycle);
                } else {
                    sConveyor.setFrontDutyCycle(0.0);
                }

                if (sOverrideBackConveyorButton.get()) {
                    sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorIntakeDutyCycle);
                } else {
                    sConveyor.setBackDutyCycle(0.0);
                }

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Intaking";
            }
        };

        mOuttaking = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorOuttakeDutyCycle);
                sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorOuttakeDutyCycle);

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Outtaking";
            }
        };

        mNudging = new IState() {
            @Override
            public void initialize() {
                nudgeStart = getFPGATimestamp();
            }

            @Override
            public IState execute() {
//                if (sConveyor.getBallCount() < 4) {
//                    sConveyor.setFrontVoltage(sConveyor.getConfig().kFrontConveyorNudgeVoltage);
//                } else {
//                    sConveyor.setFrontVoltage(0.0);
//                }

                if (sConveyor.getBallCount() > 3) {
                    return mIdle;
                }

                sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorNudgeDutyCycle);
                sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorNudgeDutyCycle);

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Nudging";
            }
        };

        mNudgingDown = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorOuttakeDutyCycle);
                sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorOuttakeDutyCycle);

                if (getFPGATimestamp() - mStartTime > sConveyor.getConfig().kNudgingDownTimeSeconds) {
                    return determineState();
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Nudging Down";
            }
        };

        mFeeding = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorFeedDutyCycle);
                sConveyor.setBackDutyCycle(sConveyor.getConfig().kBackConveyorFeedDutyCycle);

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Feeding";
            }
        };

        mStateMachine = new StateMachine("Conveyor", mIdle);

        sResetBallCountButton.whenPressed(sConveyor::resetBallCount);
    }

    private IState determineState() {
        if (SubsystemFlags.getInstance().isReadyToShoot()) {
            return mFeeding;
        }

        if (SubsystemFlags.getInstance().isOuttaking()) {
            return mOuttaking;
        }

        if (((getFPGATimestamp() - nudgeStart < sConveyor.getConfig().kNudgeTimeSeconds
                && getFPGATimestamp() - nudgeStart > 0.02)
                || sConveyor.shouldNudge()) && sConveyor.getBallCount() <= 3) {
            return mNudging;
        }

        if (sNudgeDownButton.isRisingEdge()
                || (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToNudgeDown())) {
            return mNudgingDown;
        }

        if (SubsystemFlags.getInstance().isIntaking()) {
            return mIntaking;
        }

        return mIdle;
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
