package frc.robot.commands.background;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.Turret;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;

import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.DriverPreferences.kTurretMasterOverrideDeadband;
import static frc.robot.Constants.DriverPreferences.kTurretScaleFactor;
import static frc.robot.Constants.Turret.kClosedLoopErrorTolerance;
import static frc.robot.Constants.Turret.kWithinToleranceLoopsToSettle;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sTurret;

public class TurretCommand extends CommandBase {

    private enum SweepingState {
        CW(-0.3),
        CCW(0.3);

        private double mDutyCycle;

        SweepingState(double dutyCycle) {
            mDutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return mDutyCycle;
        }
    }

    private final double kZeroingDutyCycle = 0.5;
    private final double kZeroingTimeout = 2.0;

    private final Rotation2d mHomeRobotRelativeHeading = Rotation2d.fromDegrees(0);
    private final Rotation2d mTargetFieldRelativeHeading = Rotation2d.fromDegrees(0);

    private final IState mIdle;
    private final IState mZeroing;
    private final IState mHoming;
    private final IState mManual;
    private final IState mAligningFromLimelight;
    private final IState mAligningFieldRelative;
    private final IState mSweeping;

    private final StateMachine mStateMachine;

    private boolean mHasZeroed;
    private boolean mIsManualControlEnabled;

    public TurretCommand() {
        addRequirements(sTurret);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sTurret.setOpenLoopDutyCycle(0);

                if (!mHasZeroed) {
                    return mZeroing;
                }

                if (sHomeTurretButton.isRisingEdge()) {
                    return mHoming;
                }

                if (isMasterOverride()) {
                    return mManual;
                }

                if (sAlignTurretButton.isRisingEdge()) {
                    LimelightHelper.setLEDMode(true);

                    if (LimelightHelper.getTV() > 0) {
                        return mAligningFromLimelight;
                    } else {
                        return mAligningFieldRelative;
                    }
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

        mZeroing = new IState() {
            private double mStartTime;

            @Override
            public void initialize() {
                mIsManualControlEnabled = false;

                mStartTime = getFPGATimestamp();
            }

            @Override
            public IState execute() {
                sTurret.setOpenLoopDutyCycle(kZeroingDutyCycle);

                if (getFPGATimestamp() - mStartTime > kZeroingTimeout) {
                    if (!mHasZeroed) {
                        DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Failed to zero");
                        mHasZeroed = true;
                    }

                    return mIdle;
                }

                if (sTurret.isForwardLimitRisingEdge()) {
                    sTurret.zero();
                    sTurret.enableSoftLimits();
                    mHasZeroed = true;
                    return mHoming;
                }

                return mZeroing;
            }

            @Override
            public void finish() {
                mIsManualControlEnabled = true;
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
                mIsManualControlEnabled = false;
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                sTurret.setRobotRelativeHeading(mHomeRobotRelativeHeading, Turret.ControlState.POSITIONAL);

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits()) < kClosedLoopErrorTolerance) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    return mIdle;
                }

                return mHoming;
            }

            @Override
            public void finish() {
                mIsManualControlEnabled = true;
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

                return mManual;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Manual";
            }
        };

        mAligningFromLimelight = new IState() {
            private Rotation2d mTargetHeading;
            private int mWithinThresholdLoops;

            @Override
            public void initialize() {
                mIsManualControlEnabled = false;
                mTargetHeading = sTurret.getCurrentRobotRelativeHeading().minus(
                        Rotation2d.fromDegrees(LimelightHelper.getTX()));
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                sTurret.setRobotRelativeHeading(mTargetHeading, Turret.ControlState.POSITIONAL);

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits()) < kClosedLoopErrorTolerance) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    return mIdle;
                }

                return mAligningFromLimelight;
            }

            @Override
            public void finish() {
//                LimelightHelper.setLEDMode(false);
                mIsManualControlEnabled = true;
            }

            @Override
            public String getName() {
                return "Aligning From Limelight";
            }
        };

        mAligningFieldRelative = new IState() {
            private int mWithinThresholdLoops;

            @Override
            public void initialize() {
                mIsManualControlEnabled = false;
                mWithinThresholdLoops = 0;
            }

            @Override
            public IState execute() {
                sTurret.setFieldRelativeHeading(mTargetFieldRelativeHeading,
                        sDrivetrain.getHeading(), Turret.ControlState.POSITIONAL);

                if (Math.abs(sTurret.getClosedLoopErrorRawUnits()) < kClosedLoopErrorTolerance) {
                    mWithinThresholdLoops++;
                } else {
                    mWithinThresholdLoops = 0;
                }

                if (mWithinThresholdLoops > kWithinToleranceLoopsToSettle) {
                    return mIdle;
                }

                return mAligningFieldRelative;
            }

            @Override
            public void finish() {
                mIsManualControlEnabled = true;
            }

            @Override
            public String getName() {
                return "Aligning Field Relative";
            }
        };

        mSweeping = new IState() {
            @Override
            public void initialize() {
                mIsManualControlEnabled = true;
            }

            @Override
            public IState execute() {
                return null;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Sweeping";
            }
        };

        mStateMachine = new StateMachine("Turret", mIdle);

        mHasZeroed = false;
        mIsManualControlEnabled = false;
    }

    private boolean isMasterOverride() {
        return mIsManualControlEnabled && Math.abs(sGamepad.getRightX()) > kTurretMasterOverrideDeadband;
    }

    @Override
    public void execute() {
        mStateMachine.run();
    }
}
