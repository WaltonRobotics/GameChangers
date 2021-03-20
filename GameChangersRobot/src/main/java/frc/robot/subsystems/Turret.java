package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.TurretConfig;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.UtilMethods;

import java.util.logging.Level;

import static frc.robot.Constants.CANBusIDs.kTurretID;
import static frc.robot.Constants.DioIDs.kTurretLimitSwitchID;
import static frc.robot.Constants.PIDSlots.kTurretMotionMagicSlot;
import static frc.robot.Constants.PIDSlots.kTurretPositionalSlot;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;

public class Turret extends SubsystemBase {

    public enum ControlState {
        DUTY_CYCLE, POSITIONAL, MOTION_MAGIC
    }

    private final TurretConfig mConfig = sCurrentRobot.getCurrentRobot().getTurretConfig();

    private final TalonSRX mTurretController = new TalonSRX(kTurretID);
    private final DigitalInput mForwardLimit = new DigitalInput(kTurretLimitSwitchID);
    private final EnhancedBoolean mForwardLimitBool = new EnhancedBoolean();

    private ControlState mControlState;
    private double mSetpoint;

    public Turret() {
        configureTurretController();
    }

    private void configureTurretController() {
        mTurretController.setNeutralMode(NeutralMode.Brake);

        mTurretController.setInverted(false);
        mTurretController.setSensorPhase(false);

        mTurretController.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        mTurretController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

        mTurretController.configNominalOutputForward(0);
        mTurretController.configNominalOutputReverse(0);
        mTurretController.configPeakOutputForward(1);
        mTurretController.configPeakOutputReverse(-1);

        // Set up limits
//        mTurretController.configForwardLimitSwitchSource(LimitSwitchSource.RemoteCANifier,
//                LimitSwitchNormal.NormallyOpen);
        mTurretController.configForwardSoftLimitThreshold(mConfig.kForwardSoftLimitRawUnits);
        mTurretController.configReverseSoftLimitThreshold(mConfig.kReverseSoftLimitRawUnits);
        mTurretController.configForwardSoftLimitEnable(true);
        mTurretController.configReverseSoftLimitEnable(true);
        mTurretController.overrideLimitSwitchesEnable(false);

        // Set up positional control
        mTurretController.config_kF(kTurretPositionalSlot, 0);
        mTurretController.config_kP(kTurretPositionalSlot, mConfig.kPositionalP);
        mTurretController.config_kI(kTurretPositionalSlot, mConfig.kPositionalI);
        mTurretController.config_kD(kTurretPositionalSlot, mConfig.kPositionalD);
        mTurretController.config_IntegralZone(kTurretPositionalSlot, mConfig.kPositionalIZone);
        mTurretController.configMaxIntegralAccumulator(kTurretPositionalSlot,
                mConfig.kPositionalMaxIntegralAccumulator);

        // Set up Motion Magic
        mTurretController.config_kF(kTurretMotionMagicSlot, mConfig.kMotionMagicF);
        mTurretController.config_kP(kTurretMotionMagicSlot, mConfig.kMotionMagicP);
        mTurretController.config_kI(kTurretMotionMagicSlot, mConfig.kMotionMagicI);
        mTurretController.config_kD(kTurretMotionMagicSlot, mConfig.kMotionMagicD);
        mTurretController.config_IntegralZone(kTurretMotionMagicSlot, mConfig.kMotionMagicIZone);
        mTurretController.configMaxIntegralAccumulator(kTurretMotionMagicSlot,
                mConfig.kMotionMagicMaxIntegralAccumulator);
        mTurretController.configMotionCruiseVelocity(mConfig.kCruiseVelocity);
        mTurretController.configMotionAcceleration(mConfig.kAcceleration);
        mTurretController.configMotionSCurveStrength(mConfig.kSCurveStrength);

        mControlState = ControlState.DUTY_CYCLE;
        mSetpoint = 0.0;
    }

    @Override
    public void periodic() {
        if (mControlState == ControlState.POSITIONAL) {
            mTurretController.set(ControlMode.Position, mSetpoint);
        } else if (mControlState == ControlState.MOTION_MAGIC) {
            mTurretController.set(ControlMode.MotionMagic, mSetpoint);
        } else if (mControlState == ControlState.DUTY_CYCLE) {
            mTurretController.set(ControlMode.PercentOutput, mSetpoint);
        }

        mForwardLimitBool.set(!mForwardLimit.get());

        SmartDashboard.putBoolean(kTurretForwardLimitStateKey, isForwardLimitClosed());
        SmartDashboard.putNumber(kTurretRobotRelativeHeadingRawUnitsKey, mTurretController.getSelectedSensorPosition());
        SmartDashboard.putNumber(kTurretRobotRelativeHeadingDegreesKey,
                getRobotRelativeHeadingFromRawUnits(mTurretController.getSelectedSensorPosition()).getDegrees());
        SmartDashboard.putNumber(kTurretAngularVelocityRawUnitsKey, mTurretController.getSelectedSensorVelocity());
        SmartDashboard.putString(kTurretControlStateKey, mControlState.name());
    }

    public boolean isForwardLimitClosed() {
        return mForwardLimitBool.get();
    }

    public boolean isForwardLimitRisingEdge() {
        return mForwardLimitBool.isRisingEdge();
    }

    public void zero() {
        mTurretController.setSelectedSensorPosition(0);
    }

    public void setOpenLoopDutyCycle(double targetDutyCycle) {
        if (mControlState != ControlState.DUTY_CYCLE) {
            mSetpoint = targetDutyCycle;
            mControlState = ControlState.DUTY_CYCLE;
        }
    }

    public void setRobotRelativeHeading(Rotation2d targetHeading, ControlState controlState) {
        double setpointRawUnits;

        setpointRawUnits = getRawUnitsFromRobotRelativeHeading(targetHeading);

        setpointRawUnits = UtilMethods.limitRange(setpointRawUnits,
                mConfig.kReverseSoftLimitRawUnits, mConfig.kForwardSoftLimitRawUnits);

        if (controlState == ControlState.POSITIONAL && mControlState != ControlState.POSITIONAL) {
            mSetpoint = setpointRawUnits;
            mTurretController.selectProfileSlot(kTurretPositionalSlot, 0);
            mControlState = ControlState.POSITIONAL;
        } else if (controlState == ControlState.MOTION_MAGIC && mControlState != ControlState.MOTION_MAGIC) {
            mSetpoint = setpointRawUnits;
            mTurretController.selectProfileSlot(kTurretMotionMagicSlot, 0);
            mControlState = ControlState.MOTION_MAGIC;
        } else {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "The desired control state " + controlState.name() + " for setting turret heading is invalid");
        }
    }

    public void setFieldRelativeHeading(Rotation2d targetHeading, Rotation2d robotFieldRelativeHeading,
                                        ControlState controlState) {
        Rotation2d robotRelativeHeading = targetHeading.minus(robotFieldRelativeHeading);

        setRobotRelativeHeading(robotRelativeHeading, controlState);
    }

    public double getClosedLoopErrorRawUnits() {
        return mTurretController.getClosedLoopError();
    }

    public double getMotionMagicErrorRawUnits() {
        if (mControlState != ControlState.MOTION_MAGIC) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Attempted to retrieve Motion Magic error while not in its control state");

            return 0.0;
        } else {
            return mTurretController.getActiveTrajectoryPosition() - mSetpoint;
        }
    }

    private double getRawUnitsFromRobotRelativeHeading(Rotation2d heading) {
        return mConfig.kTicksPerDegree * getRawAngleDegreesFromRobotRelativeHeading(heading);
    }

    private double getRawAngleDegreesFromRobotRelativeHeading(Rotation2d heading) {
        return heading.getDegrees() - mConfig.kLimitSwitchPosition.getDegrees();
    }

    private Rotation2d getRobotRelativeHeadingFromRawUnits(double rawUnits) {
        return Rotation2d.fromDegrees(rawUnits / mConfig.kTicksPerDegree).plus(mConfig.kLimitSwitchPosition);
    }
}
