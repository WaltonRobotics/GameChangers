package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.vision.LimelightHelper;

import java.util.function.BooleanSupplier;
import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.Field.kEndOfZoneOneFromTargetFeet;
import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import static frc.robot.Constants.PIDSlots.kShooterShootingSlot;
import static frc.robot.Constants.PIDSlots.kShooterSpinningUpSlot;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SmartDashboardKeys.kShooterTuningSetpointRawUnitsKey;
import static frc.robot.OI.*;
import static frc.robot.Robot.sShooter;

public class ShooterCommand extends CommandBase {

    private double mSetpointRawUnits = kDefaultVelocityRawUnits;

    private final IState mIdle;
    private final IState mDeterminingSetpoint;
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
                    if (kIsInTuningMode) {
                        mSetpointRawUnits =
                                SmartDashboard.getNumber(kShooterTuningSetpointRawUnitsKey, kDefaultVelocityRawUnits);

                        return mSpinningUp;
                    }

                    return mDeterminingSetpoint;
                }

                if (mNeedsToBarf.getAsBoolean()) {
                    mSetpointRawUnits = kBarfVelocityRawUnits;

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

        mDeterminingSetpoint = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                LimelightHelper.setLEDMode(true);
                LimelightHelper.setPipeline(kAlignmentPipeline);

                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                if (LimelightHelper.getTV() > 0 || getFPGATimestamp() - mStartTime > kMaximumLEDWaitTimeSeconds) {
                    sShooter.setAdjustableHoodUp(LimelightHelper.getDistanceToTargetFeet() < kEndOfZoneOneFromTargetFeet);

                    mSetpointRawUnits = getEstimatedVelocityFromTarget();

                    return mSpinningUp;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Determining Setpoint";
            }
        };

        mSpinningUp = new IState() {
            @Override
            public void initialize() {
                SubsystemFlags.getInstance().setIsShooting(true);

                DebuggingLog.getInstance().getLogger().log(Level.FINE, "Shooter velocity setpoint: "
                        + mSetpointRawUnits);

                SmartDashboard.putNumber("Shooter setpoint", mSetpointRawUnits);

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
                SubsystemFlags.getInstance().setIsReadyToShoot(true);
                
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                // Hold the previous setpoint for a short period so Power Cells don't go short
                sShooter.setClosedLoopVelocityRawUnits(mSetpointRawUnits);

                if (getFPGATimestamp() - mStartTime > kSpinDownTimeSeconds) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {
                SubsystemFlags.getInstance().setIsShooting(false);
                SubsystemFlags.getInstance().setIsReadyToShoot(false);

                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Spinning Down";
            }
        };

        mStateMachine = new StateMachine("Shooter", mIdle);

        sToggleLimelightLEDsButton.whenPressed(LimelightHelper::toggleLimelight);
        sShooterAdjustableToggleHoodButton.whenPressed(sShooter::toggleAdjustableHood);
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getEstimatedVelocityFromTarget() {
        // If the limelight does not see a target, we use the last known "ty" value since
        // LimelightHelper uses a MovingAverage to keep track of it at all times

        if (LimelightHelper.getTV() <= 0) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "No target found for shooter. Using last known information");
        }

        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();

        distanceFeet = UtilMethods.limitRange(distanceFeet, kAbsoluteShootingDistanceFloorFeet,
                kAbsoluteShootingDistanceCeilingFeet);

        if (kUseInterpolationMap) {
            InterpolatingDouble result;

            if (distanceFeet < kEndOfZoneOneFromTargetFeet) {
                result = sShooter.getConfig().kZoneOneShooterMap.getInterpolated(new InterpolatingDouble(distanceFeet));
            } else {
                result = sShooter.getConfig().kOtherZonesShooterMap.getInterpolated(new InterpolatingDouble(distanceFeet));
            }

            if (result != null) {
                return result.value;
            } else {
                return sShooter.getConfig().kOtherZonesShooterMap.getInterpolated(new InterpolatingDouble(kDefaultShootingDistanceFeet)).value;
            }
        } else {
            if (distanceFeet < kEndOfZoneOneFromTargetFeet) {
                return sShooter.getConfig().kZoneOneShooterPolynomial.predict(distanceFeet);
            }

            return sShooter.getConfig().kOtherZonesShooterPolynomial.predict(distanceFeet);
        }
    }

}
