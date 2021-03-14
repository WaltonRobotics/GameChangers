package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;

import java.util.function.BooleanSupplier;
import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.PIDSlots.kShooterShootingSlot;
import static frc.robot.Constants.PIDSlots.kShooterSpinningUpSlot;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SmartDashboardKeys.kShooterTuningSetpointRawUnitsKey;
import static frc.robot.OI.sBarfButton;
import static frc.robot.OI.sShootButton;
import static frc.robot.Robot.sShooter;

public class ShooterCommand extends CommandBase {

    private double mSetpointRawUnits = 12500;

    private final IState mIdle;
    private final IState mSpinningUp;
    private final IState mShooting;
    private final IState mSpinningDown;

    private final StateMachine mStateMachine;

    private final BooleanSupplier mNeedsToShoot = () -> (sShootButton.get());
    private final BooleanSupplier mNeedsToBarf = () -> (sBarfButton.get());

    public ShooterCommand() {
        addRequirements(sShooter);

        mIdle = new IState() {
            @Override
            public void initialize() {
                sShooter.setOpenLoopDutyCycle(0);
            }

            @Override
            public IState execute() {
                sShooter.setOpenLoopDutyCycle(0);

                if (mNeedsToShoot.getAsBoolean()) {
                    LimelightHelper.setLEDMode(true);

                    if (kIsInTuningMode) {
                        mSetpointRawUnits = SmartDashboard.getNumber(kShooterTuningSetpointRawUnitsKey, kDefaultVelocityRawUnits);
                    } else {
                        mSetpointRawUnits = sShooter.getEstimatedVelocityFromTarget();
                    }

                    return mSpinningUp;
                }

                if (mNeedsToBarf.getAsBoolean()) {
                    return mSpinningUp;
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

        mSpinningUp = new IState() {
            @Override
            public void initialize() {
                DebuggingLog.getInstance().getLogger().log(Level.FINE, "Shooter velocity setpoint: "
                        + mSetpointRawUnits);

                sShooter.setProfileSlot(kShooterSpinningUpSlot);
            }

            @Override
            public IState execute() {
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (!mNeedsToShoot.getAsBoolean() && !mNeedsToBarf.getAsBoolean()) {
                    return mSpinningDown;
                }

                if (Math.abs(sShooter.getClosedLoopErrorRawUnits()) <= kSpinningUpToleranceRawUnits) {
                    return mShooting;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Spinning Up";
            }
        };

        mShooting = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIsReadyToShoot(true);

                sShooter.setProfileSlot(kShooterShootingSlot);
            }

            @Override
            public IState execute() {
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (!mNeedsToShoot.getAsBoolean() && !mNeedsToBarf.getAsBoolean()) {
                    return mSpinningDown;
                }

                if (Math.abs(sShooter.getClosedLoopErrorRawUnits()) > kShootingToleranceRawUnits) {
                    return mSpinningUp;
                }

                return this;
            }

            @Override
            public void finish() {
                SubsystemFlags.getInstance().setIsReadyToShoot(false);
            }

            @Override
            public String getName() {
                return "Shooting";
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
                SubsystemFlags.getInstance().setIsReadyToShoot(false);

                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Spinning Down";
            }
        };

        mStateMachine = new StateMachine("Shooter", mIdle);
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
