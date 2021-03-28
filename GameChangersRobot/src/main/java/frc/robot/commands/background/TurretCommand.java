package frc.robot.commands.background;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;
import frc.robot.subsystems.Turret;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;
import frc.robot.vision.LimelightHelper;

import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.DriverPreferences.kTurretMasterOverrideDeadband;
import static frc.robot.Constants.DriverPreferences.kTurretScaleFactor;
import static frc.robot.Constants.Field.kTargetFieldRelativeHeading;
import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import static frc.robot.Constants.Turret.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sTurret;

public class TurretCommand extends CommandBase {

    // TODO: Add timeouts on all auto-align functions

    private final Rotation2d kHomeRobotRelativeHeading = Rotation2d.fromDegrees(0);

    private final IState mIdle;
    private final IState mZeroing;
    private final IState mHoming;
    private final IState mManual;
    private final IState mDeterminingAlignmentMethod;
    private final IState mAligningFromLimelightTX;
    private final IState mAligningFromLimelightClosedLoop;
    private final IState mAligningFieldRelative;
    private final IState mLockingSetpoint;

    private final StateMachine mStateMachine;

    private boolean mHasZeroed;

    public TurretCommand() {
        addRequirements(sTurret);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sTurret.setOpenLoopDutyCycle(0);

                if (isMasterOverride()) {
                    return mManual;
                }

                if (sAlignTurretButton.get()) {
                    return mDeterminingAlignmentMethod;
                }

                if (!mHasZeroed) {
                    return mZeroing;
                }

                if (sHomeTurretButton.get()) {
                    return mHoming;
                }

                if (SubsystemFlags.getInstance().isShooting()) {
                    return mLockingSetpoint;
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
                    }

                    return mIdle;
                }

                if (sTurret.isForwardLimitRisingEdge()) {
                    sTurret.zero();
                    sTurret.enableSoftLimits();
                    mHasZeroed = true;
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

            @Override
            public void initialize() {
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                sTurret.setRobotRelativeHeading(kHomeRobotRelativeHeading, Turret.ControlState.POSITIONAL);

                if (isMasterOverride()) {
                    return mManual;
                }

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits()) < kClosedLoopErrorToleranceDegrees) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
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
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                if (!isMasterOverride()) {
                    return mIdle;
                }

                sTurret.setOpenLoopDutyCycle(sGamepad.getRightX() * kTurretScaleFactor);

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

                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                if (LimelightHelper.getTV() > 0) {
                    if (!mHasZeroed) {
                        return mAligningFromLimelightClosedLoop;
                    } else {
                        return mAligningFromLimelightTX;
                    }
                } else {
                    if (getFPGATimestamp() - mStartTime > kMaximumLEDWaitTimeSeconds) {
                        if (mHasZeroed) {
                            return mAligningFieldRelative;
                        }
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

            @Override
            public void initialize() {
                mTargetHeading = sTurret.getCurrentRobotRelativeHeading().minus(
                        Rotation2d.fromDegrees(LimelightHelper.getTX()));
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                if (isMasterOverride()) {
                    return mManual;
                }

                sTurret.setRobotRelativeHeading(mTargetHeading, Turret.ControlState.POSITIONAL);

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits())
                        < kClosedLoopErrorToleranceDegrees * sTurret.getConfig().kTicksPerDegree) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    return mIdle;
                }

                return this;
            }

            @Override
            public void finish() {
                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning From Limelight";
            }
        };

        mAligningFromLimelightClosedLoop = new IState() {
            private int mWithinThresholdLoops;

            @Override
            public void initialize() {
                sTurret.getClosedLoopAutoAlignProfiledPID().reset(
                        new TrapezoidProfile.State(
                                UtilMethods.restrictAngle(sTurret.getCurrentRobotRelativeHeading().getDegrees(), -180.0, 180.0),
                                sTurret.getCurrentAngularVelocityDegreesPerSec()
                        )
                );

                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                if (isMasterOverride()) {
                    return mManual;
                }

                if (LimelightHelper.getTV() > 0) {
                    double headingError = -LimelightHelper.getTX();

                    double currentHeading = UtilMethods.restrictAngle(
                            sTurret.getCurrentRobotRelativeHeading().getDegrees(), -180.0, 180.0
                    );

                    double turnRate = sTurret.getClosedLoopAutoAlignProfiledPID().calculate(
                            currentHeading,
                            currentHeading + headingError
                    );

                    sTurret.setOpenLoopDutyCycle(turnRate);

//                    // Alternate alignment method
//                    double turnRate = 0.0;
//
//                    if (tx >= kMinimumAimThresholdDegrees) {
//                        turnRate = kAimingKp * headingError - kMinimumAimDutyCycle;
//                    } else if (tx < kMinimumAimThresholdDegrees) {
//                        turnRate = kAimingKp * headingError + kMinimumAimDutyCycle;
//                    }
//
//                    sTurret.setOpenLoopDutyCycle(turnRate);

                    if (Math.abs(headingError) < kClosedLoopErrorToleranceDegrees) {
                        mWithinThresholdLoops++;
                    } else {
                        mWithinThresholdLoops = 0;
                    }

                    if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                        return mIdle;
                    }
                } else {
                    sTurret.setOpenLoopDutyCycle(0.0);
                }

                return this;
            }

            @Override
            public void finish() {
                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning From Limelight Closed Loop";
            }
        };

        mAligningFieldRelative = new IState() {
            private int mWithinThresholdLoops;

            @Override
            public void initialize() {
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                sTurret.setFieldRelativeHeading(kTargetFieldRelativeHeading,
                        sDrivetrain.getHeading().plus(Rotation2d.fromDegrees(180.0)), Turret.ControlState.POSITIONAL);

                if (isMasterOverride()) {
                    return mManual;
                }

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits()) <
                        kClosedLoopErrorToleranceDegrees * sTurret.getConfig().kTicksPerDegree) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    if (LimelightHelper.getTV() > 0) {
                        return mAligningFromLimelightTX;
                    } else {
                        return mIdle;
                    }
                }

                return mAligningFieldRelative;
            }

            @Override
            public void finish() {
                LimelightHelper.setLEDMode(false);
            }

            @Override
            public String getName() {
                return "Aligning Field Relative";
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

    private boolean isMasterOverride() {
        return Math.abs(sGamepad.getRightX()) > kTurretMasterOverrideDeadband;
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }
}
