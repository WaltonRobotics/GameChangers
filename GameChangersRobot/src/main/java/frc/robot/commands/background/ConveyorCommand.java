package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Conveyor.kFrontLoadingCapacity;
import static frc.robot.Constants.Conveyor.kMaximumBallCapacity;
import static frc.robot.OI.*;
import static frc.robot.Robot.sConveyor;

public class ConveyorCommand extends CommandBase {

    private final IState mIdle;
    private final IState mIntaking;
    private final IState mOuttaking;
    private final IState mNudging;
    private final IState mFeeding;

    private final StateMachine mStateMachine;

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
//                if (sConveyor.getBallCount() < 4) {
//                    sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorIntakeDutyCycle);
//                } else {
//                    sConveyor.setFrontDutyCycle(0.0);
//                }

                sConveyor.setFrontDutyCycle(sConveyor.getConfig().kFrontConveyorIntakeDutyCycle);

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
            private double mStartTime;

            @Override
            public void initialize() {
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
//                if (sConveyor.getBallCount() < 4) {
//                    sConveyor.setFrontVoltage(sConveyor.getConfig().kFrontConveyorNudgeVoltage);
//                } else {
//                    sConveyor.setFrontVoltage(0.0);
//                }

                sConveyor.setFrontVoltage(sConveyor.getConfig().kFrontConveyorNudgeVoltage);
                sConveyor.setBackVoltage(sConveyor.getConfig().kBackConveyorNudgeVoltage);

                if (getFPGATimestamp() - mStartTime > sConveyor.getConfig().kNudgeTimeSeconds) {
                    return determineState();
                }

                return mNudging;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Nudging";
            }
        };

        mFeeding = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sConveyor.setFrontVoltage(sConveyor.getConfig().kFrontConveyorFeedVoltage);
                sConveyor.setBackVoltage(sConveyor.getConfig().kBackConveyorFeedVoltage);

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

        if (sConveyor.shouldNudge()) {
            return mNudging;
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
