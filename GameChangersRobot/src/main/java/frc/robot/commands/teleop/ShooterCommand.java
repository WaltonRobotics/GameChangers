package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.OI.shootButton;
import static frc.robot.Robot.sShooter;

public class ShooterCommand extends CommandBase {

    private double kToleranceRawUnits = 100;
    private double kSpinDownTime = 0.25;

    private double mSetpointRawUnits = 12500;

    private IState mIdle;
    private IState mSpinningUp;
    private IState mShooting;
    private IState mSpinningDown;

    private StateMachine mStateMachine;

    private final BooleanSupplier mNeedsToShoot = () -> (shootButton.get());

    public ShooterCommand() {
        addRequirements(sShooter);

        configureStateMachine();
    }

    private void configureStateMachine() {
        mIdle = new IState() {
            @Override
            public void initialize() {
                sShooter.setOpenLoopDutyCycles(0);
            }

            @Override
            public IState execute() {
                sShooter.setOpenLoopDutyCycles(0);

                if (shootButton.get()) {
                    return mSpinningUp;
                }

                return this;
            }

            @Override
            public void finish() {
            }
        };

        mSpinningUp = new IState() {
            @Override
            public void initialize() {
                sShooter.setProfileSlot(0);
            }

            @Override
            public IState execute() {
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (!mNeedsToShoot.getAsBoolean()) {
                    return mSpinningDown;
                }

                if (Math.abs(sShooter.getClosedLoopErrorRawUnits()) <= kToleranceRawUnits) {
                    return mShooting;
                }

                return this;
            }

            @Override
            public void finish() {
            }
        };

        mShooting = new IState() {
            @Override
            public void initialize() {
                sShooter.setProfileSlot(1);
            }

            @Override
            public IState execute() {
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (!mNeedsToShoot.getAsBoolean()) {
                    return mSpinningDown;
                }

                if (Math.abs(sShooter.getClosedLoopErrorRawUnits()) > kToleranceRawUnits) {
                    return mSpinningUp;
                }

                return this;
            }

            @Override
            public void finish() {

            }
        };

        mSpinningDown = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                // Hold the previous setpoint for a short period so Power Cells don't go short
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (getFPGATimestamp() - mStartTime > kSpinDownTime) {
                    return mIdle;
                }

                return this;
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
