package frc.robot.commands.background;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.AutonFlags;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;
import frc.robot.subsystems.Turret;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;

import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.DriverPreferences.kTurretMasterOverrideDeadband;
import static frc.robot.Constants.DriverPreferences.kTurretScaleFactor;
import static frc.robot.Constants.Field.kTargetFieldRelativeHeading;
import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import static frc.robot.Constants.SmartDashboardKeys.kTurretIsHomedForClimbingKey;
import static frc.robot.Constants.Turret.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sTurret;

public class TurretCommand extends CommandBase {

    private static boolean mHasZeroed;
    private final Rotation2d kHomeRobotRelativeHeading = Rotation2d.fromDegrees(0);
    private final IState mIdle;
    private final IState mZeroing;
    private final IState mHoming;
    private final IState mManual;
    private final IState mDeterminingAlignmentMethod;
    private final IState mAligningFromLimelightTX;
    private final IState mAligningFromLimelightClosedLoop;
    private final IState mAligningFieldRelative;
    private final IState mAutonAligningFieldRelative;
    private final IState mLockingSetpoint;
    private final StateMachine mStateMachine;

    public TurretCommand() {
        addRequirements(sTurret);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                if (!SubsystemFlags.getInstance().isClimberDeployed()) {
                    if (isMasterOverride()) {
                        return mManual;
                    }

                    if (sAlignTurretButton.isRisingEdge()
                            || (AutonFlags.getInstance().isInAuton()
                            && AutonFlags.getInstance().doesAutonNeedToAlignTurret())) {
                        return mDeterminingAlignmentMethod;
                    }

                    if (AutonFlags.getInstance().isInAuton()) {
                        if (!mHasZeroed && AutonFlags.getInstance().isAutonTurretZeroingEnabled()) {
                            return mZeroing;
                        }
                    } else {
                        if (!mHasZeroed || sZeroTurretButton.isRisingEdge() &&
                                !SubsystemFlags.getInstance().isTurretZeroingDisabled()) {
                            return mZeroing;
                        }
                    }

                    if (AutonFlags.getInstance().isInAuton()
                            && AutonFlags.getInstance().doesAutonNeedToAlignTurretFieldRelative()) {
                        return mAutonAligningFieldRelative;
                    }

                    if (sHomeTurretButton.isRisingEdge()
                            || SubsystemFlags.getInstance().doesTurretNeedToHomeForClimbing()) {
                        return mHoming;
                    }

                    if (SubsystemFlags.getInstance().isShooting()) {
                        return mLockingSetpoint;
                    }
                }

                sTurret.setOpenLoopDutyCycle(0);

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

        mZeroing = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                sTurret.setOpenLoopDutyCycle(kZeroingDutyCycle);

                if (getFPGATimestamp() - mStartTime > kZeroingTimeout) {
                    if (!mHasZeroed) {
                        DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Failed to zero");
                        SubsystemFlags.getInstance().setTurretHasZeroed(false);
                    }

                    return mIdle;
                }

                if (sTurret.isForwardLimitClosed()) {
                    sTurret.zero();
                    sTurret.enableSoftLimits();
                    mHasZeroed = true;
                    SubsystemFlags.getInstance().setTurretHasZeroed(true);
                    return mHoming;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Zeroing";
            }
        };

        mHoming = new IState() {
            private int mWithinThresholdLoops;
            private double mStartTime;

            @Override
            public void initialize() {
                mWithinThresholdLoops = 0;
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                sTurret.setRobotRelativeHeading(kHomeRobotRelativeHeading, Turret.ControlState.MOTION_MAGIC);

                if (isMasterOverride()) {
                    return mManual;
                }

                if (Math.abs(sTurret.getMotionMagicErrorDegrees()) < kPositionClosedLoopErrorToleranceDegrees) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    return mIdle;
                }

                if (AutonFlags.getInstance().isInAuton() && AutonFlags.getInstance().doesAutonNeedToAlignTurret()) {
                    return mIdle;
                }

                if (!SubsystemFlags.getInstance().doesTurretNeedToHomeForClimbing()
                        && getFPGATimestamp() - mStartTime > kHomingTimeout) {
                    return mIdle;
                }

                if (SubsystemFlags.getInstance().doesTurretNeedToHomeForClimbing()
                        && SubsystemFlags.getInstance().isTurretHomedForClimbing()) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Homing";
            }
        };

        mManual = new IState() {
            private double lastTx;
            private double lastTime;

            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                if (!isMasterOverride()) {
                    return mIdle;
                }

                double currentTx = LimelightHelper.getTX();
                double currentTime = Timer.getFPGATimestamp();

                SmartDashboard.putNumber("Turret velocity", (currentTx - lastTx) / (currentTime - lastTime));

                lastTime = currentTime;
                lastTx = currentTx;

                sTurret.setOpenLoopDutyCycle(-sManipulationGamepad.getRightX() * kTurretScaleFactor);

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Manual";
            }
        };

        mDeterminingAlignmentMethod = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                LimelightHelper.setLEDMode(true);
                LimelightHelper.setPipeline(kAlignmentPipeline);

                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                if (LimelightHelper.getTV() > 0) {
                    return mAligningFromLimelightClosedLoop;
                } else {
                    if (getFPGATimestamp() - mStartTime > kMaximumLEDWaitTimeSeconds) {
                        if (mHasZeroed) {
                            return mAligningFieldRelative;
                        }

                        return mIdle;
                    }
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Determining Alignment Method";
            }
        };

        mAligningFromLimelightTX = new IState() {

            private Rotation2d mTargetHeading;
            private int mWithinThresholdLoops;
            private double mStartTime;

            @Override
            public void initialize() {
                mTargetHeading = sTurret.getCurrentRobotRelativeHeading().minus(
                        Rotation2d.fromDegrees(LimelightHelper.getTX()));
                mWithinThresholdLoops = 0;
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                if (isMasterOverride()) {
                    return mManual;
                }

                sTurret.setRobotRelativeHeading(mTargetHeading, Turret.ControlState.MOTION_MAGIC);

                if (Math.abs(sTurret.getMotionMagicErrorDegrees())
                        < kPositionClosedLoopErrorToleranceDegrees * sTurret.getConfig().kTicksPerDegree) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle
                        || getFPGATimestamp() - mStartTime > kAlignmentTimeout) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {
//                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning From Limelight TX";
            }
        };

        mAligningFromLimelightClosedLoop = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                sTurret.getClosedLoopAutoAlignProfiledPID().reset(
                        new TrapezoidProfile.State(
                                LimelightHelper.getTX(),
                                0.0
                        )
                );

                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                if (isMasterOverride()) {
                    return mManual;
                }

                if (AutonFlags.getInstance().isInAuton()) {
                    if (!AutonFlags.getInstance().doesAutonNeedToAlignTurret()) {
                        return mIdle;
                    }
                } else {
                    if (!sAlignTurretButton.get()) {
                        return mIdle;
                    }
                }

                if (LimelightHelper.getTV() > 0) {
                    double tx = LimelightHelper.getTX();
                    double headingError = tx;

                    SmartDashboard.putNumber("Heading error", headingError);

                    if (Math.abs(headingError) < kAlignedThresholdDegrees) {
                        return mIdle;
                    }

                    double turnRate = sTurret.getClosedLoopAutoAlignProfiledPID().calculate(headingError);

                    if (Math.abs(headingError) > kMinimumAimThresholdDegrees) {
                        // Turret is to the left of the target
                        if (turnRate > 0) {
                            turnRate += kMinimumAimDutyCycleCW;
                        } else {
                            // Turret is to the right of the target
                            turnRate -= kMinimumAimDutyCycleCCW;
                        }
                    }

                    sTurret.setOpenLoopDutyCycle(turnRate);

//                    // Alternate alignment method
//                    double turnRate = 0.0;
//
//                    if (tx > 0 && tx < kMinimumAimThresholdDegrees) {
//                        turnRate = kAimingKp * headingError - kMinimumAimDutyCycle;
//                    } else if (tx < 0 && tx > kMinimumAimThresholdDegrees) {
//                        turnRate = kAimingKp * headingError + kMinimumAimDutyCycle;
//                    } else {
//                        turnRate = kAimingKp * headingError;
//                    }
//
//                    turnRate = kAimingKp * headingError;
//
//                    if (Math.abs(turnRate) < kMinimumAimDutyCycle) {
//                        if (turnRate < 0) {
//                            turnRate -= kMinimumAimDutyCycle;
//                        } else {
//                            turnRate += kMinimumAimDutyCycle;
//                        }
//                    }
//
//                    sTurret.setOpenLoopDutyCycle(turnRate);
                } else {
                    sTurret.setOpenLoopDutyCycle(0.0);
                }

                return this;
            }

            @Override
            public void finish() {
//                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning From Limelight Closed Loop";
            }
        };

        mAligningFieldRelative = new IState() {
            private int mWithinThresholdLoops;
            private double mStartTime;

            @Override
            public void initialize() {
                mWithinThresholdLoops = 0;
                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                sTurret.setFieldRelativeHeading(kTargetFieldRelativeHeading,
                        sDrivetrain.getHeading(), Turret.ControlState.MOTION_MAGIC);

                if (isMasterOverride()) {
                    return mManual;
                }

                if (Math.abs(sTurret.getMotionMagicErrorDegrees()) <
                        kPositionClosedLoopErrorToleranceDegrees * sTurret.getConfig().kTicksPerDegree) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle || getFPGATimestamp() - mStartTime > kAlignmentFieldRelativeTimeout) {
                    if (LimelightHelper.getTV() > 0) {
                        return mAligningFromLimelightClosedLoop;
                    } else {
                        return mIdle;
                    }
                }

                return mAligningFieldRelative;
            }

            @Override
            public void finish() {
//                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning Field Relative";
            }
        };

        mAutonAligningFieldRelative = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sTurret.setFieldRelativeHeading(kTargetFieldRelativeHeading,
                        sDrivetrain.getHeading(), Turret.ControlState.MOTION_MAGIC);

                if (!(AutonFlags.getInstance().isInAuton()
                        && AutonFlags.getInstance().doesAutonNeedToAlignTurretFieldRelative())) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Auton Aligning Field Relative";
            }
        };

        mLockingSetpoint = new IState() {
            private Rotation2d mLockSetpoint;

            @Override
            public void initialize() {
                mLockSetpoint = sTurret.getCurrentRobotRelativeHeading();
            }

            @Override
            public IState execute() {
                sTurret.setRobotRelativeHeading(mLockSetpoint, Turret.ControlState.POSITIONAL);

                if (!SubsystemFlags.getInstance().isShooting()) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Locking Setpoint";
            }
        };

        mStateMachine = new StateMachine("Turret", mIdle);

        mHasZeroed = false;
    }

    public static void setHasZeroed(boolean flag) {
        mHasZeroed = flag;
    }

    private boolean isMasterOverride() {
        return Math.abs(sManipulationGamepad.getRightX()) > kTurretMasterOverrideDeadband;
    }

    public boolean hasZeroed() {
        return mHasZeroed;
    }

    @Override
    public void execute() {
        boolean isHomedForClimbing = Math.abs(kHomeRobotRelativeHeading.minus(
                sTurret.getCurrentRobotRelativeHeading()).getDegrees()) < kClimbingHomedToleranceDegrees;

        SubsystemFlags.getInstance().setIsTurretHomedForClimbing(isHomedForClimbing);

        SmartDashboard.putBoolean(kTurretIsHomedForClimbingKey, isHomedForClimbing);

        mStateMachine.run();
    }

}
