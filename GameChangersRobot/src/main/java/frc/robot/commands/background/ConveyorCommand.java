package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Conveyor.kMaximumBallCapacity;
import static frc.robot.Constants.Conveyor.kNudgeTime;
import static frc.robot.OI.sOverrideBackConveyorButton;
import static frc.robot.OI.sOverrideFrontConveyorButton;
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
                    sConveyor.setFrontDutyCycles(1.0);
                } else {
                    sConveyor.setFrontDutyCycles(0.0);
                }

                if (sOverrideBackConveyorButton.get()) {
                    sConveyor.setBackDutyCycles(1.0);
                } else {
                    sConveyor.setBackDutyCycles(0.0);
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
                if (sConveyor.getBallCount() < kMaximumBallCapacity) {
                    sConveyor.setFrontDutyCycles(1.0);
                } else {
                    sConveyor.setFrontDutyCycles(0.0);
                }

                if (sOverrideBackConveyorButton.get()) {
                    sConveyor.setBackDutyCycles(1.0);
                } else {
                    sConveyor.setBackDutyCycles(0.0);
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
                sConveyor.setFrontDutyCycles(-1.0);
                sConveyor.setBackDutyCycles(-1.0);

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return null;
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
                sConveyor.setFrontDutyCycles(1.0);
                sConveyor.setBackDutyCycles(1.0);

                if (getFPGATimestamp() - mStartTime > kNudgeTime) {
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
                sConveyor.setFrontDutyCycles(1.0);
                sConveyor.setBackDutyCycles(1.0);

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

}