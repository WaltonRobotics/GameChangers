package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ShooterConfig;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;
import frc.robot.vision.LimelightHelper;

import java.util.Arrays;
import java.util.logging.Level;

import static frc.robot.Constants.CANBusIDs.kFlywheelMasterID;
import static frc.robot.Constants.CANBusIDs.kFlywheelSlaveID;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.PIDSlots.kShooterShootingSlot;
import static frc.robot.Constants.PIDSlots.kShooterSpinningUpSlot;
import static frc.robot.Constants.PneumaticsIDs.kAdjustableHoodSolenoidID;
import static frc.robot.Constants.Shooter.kFlywheelDiameterInches;
import static frc.robot.Constants.Shooter.kFlywheelEncoderPPR;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;

public class Shooter extends SubsystemBase {

    private final ShooterConfig mConfig = sCurrentRobot.getCurrentRobot().getShooterConfig();

    private final TalonFX mFlywheelMaster = new TalonFX(kFlywheelMasterID);
    private final TalonFX mFlywheelSlave = new TalonFX(kFlywheelSlaveID);
    private final Solenoid mAdjustableHoodSolenoid = new Solenoid(kAdjustableHoodSolenoidID);

    public Shooter() {
        configureFlywheelControllers();
    }

    private void configureFlywheelControllers() {
        mFlywheelMaster.setNeutralMode(NeutralMode.Coast);
        mFlywheelSlave.setNeutralMode(NeutralMode.Coast);

        mFlywheelMaster.setInverted(true);
        mFlywheelMaster.setSensorPhase(true);
        mFlywheelSlave.setInverted(false);
        mFlywheelSlave.setSensorPhase(false);
        mFlywheelSlave.follow(mFlywheelMaster);

        mFlywheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
        mFlywheelMaster.configNominalOutputForward(0);
        mFlywheelMaster.configNominalOutputReverse(0);
        mFlywheelMaster.configPeakOutputForward(1);
        mFlywheelMaster.configPeakOutputReverse(-1);

//        TODO: Check the following two settings by scheduling the ShooterMeasurementTuning command
//        mFlywheelMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
//        mFlywheelMaster.configVelocityMeasurementWindow(32);

        mFlywheelMaster.config_kF(kShooterSpinningUpSlot, mConfig.kSpinningUpF);
        mFlywheelMaster.config_kP(kShooterSpinningUpSlot, mConfig.kSpinningUpP);
        mFlywheelMaster.config_kI(kShooterSpinningUpSlot, mConfig.kSpinningUpI);
        mFlywheelMaster.config_kD(kShooterSpinningUpSlot, mConfig.kSpinningUpD);
        mFlywheelMaster.configAllowableClosedloopError(kShooterSpinningUpSlot, 0);
        mFlywheelMaster.config_IntegralZone(kShooterSpinningUpSlot, mConfig.kSpinningUpIZone);
        mFlywheelMaster.configMaxIntegralAccumulator(kShooterSpinningUpSlot, mConfig.kSpinningUpMaxIntegralAccumulator);
        mFlywheelMaster.configClosedLoopPeakOutput(kShooterSpinningUpSlot, 1.0);

        mFlywheelMaster.config_kF(kShooterShootingSlot, mConfig.kShootingF);
        mFlywheelMaster.config_kP(kShooterShootingSlot, mConfig.kShootingP);
        mFlywheelMaster.config_kI(kShooterShootingSlot, mConfig.kShootingI);
        mFlywheelMaster.config_kD(kShooterShootingSlot, mConfig.kShootingD);
        mFlywheelMaster.configAllowableClosedloopError(kShooterShootingSlot, 0);
        mFlywheelMaster.config_IntegralZone(kShooterShootingSlot, mConfig.kShootingIZone);
        mFlywheelMaster.configMaxIntegralAccumulator(kShooterShootingSlot, mConfig.kShootingMaxIntegralAccumulator);
        mFlywheelMaster.configClosedLoopPeakOutput(kShooterShootingSlot, 1.0);

        // Voltage compensation
        mFlywheelMaster.configVoltageCompSaturation(mConfig.kMaxVoltage);
        mFlywheelMaster.enableVoltageCompensation(true);
        mFlywheelSlave.configVoltageCompSaturation(mConfig.kMaxVoltage);
        mFlywheelSlave.enableVoltageCompensation(false);
    }

    public void setProfileSlot(int profileSlot) {
        mFlywheelMaster.selectProfileSlot(profileSlot, 0);
    }

    public void setOpenLoopDutyCycle(double targetDutyCycle) {
        mFlywheelMaster.set(ControlMode.PercentOutput, targetDutyCycle);
        mFlywheelSlave.set(TalonFXControlMode.Follower, kFlywheelMasterID);
    }

    public void setClosedLoopVelocityRawUnits(double targetVelocity) {
        mFlywheelMaster.set(ControlMode.Velocity, targetVelocity);
        mFlywheelSlave.set(TalonFXControlMode.Follower, kFlywheelMasterID);
    }

    public double getVelocityRawUnits() {
        return mFlywheelMaster.getSelectedSensorVelocity();
    }

    public double getClosedLoopErrorRawUnits() {
        return mFlywheelMaster.getClosedLoopError();
    }

    public double getClosedLoopErrorRevolutionsPerSec() {
        return getClosedLoopErrorRawUnits() / kFlywheelEncoderPPR * 10.;
    }

    public double getClosedLoopErrorInchesPerSec() {
        return getClosedLoopErrorRevolutionsPerSec() * kFlywheelDiameterInches;
    }

    public boolean isAdjustableHoodUp() {
        return mAdjustableHoodSolenoid.get();
    }

    public void setAdjustableHoodUp(boolean isUp) {
        mAdjustableHoodSolenoid.set(isUp);
    }

    public void toggleAdjustableHood() {
        mAdjustableHoodSolenoid.toggle();
    }

    @Override
    public void periodic() {
        LimelightHelper.updateData();

        if (kIsInTuningMode) {
            mFlywheelMaster.configVelocityMeasurementPeriod(
                    VelocityMeasPeriod.valueOf(SmartDashboard.getNumber(kShooterMeasurementPeriodKey, 1)));
            mFlywheelMaster.configVelocityMeasurementWindow(
                    (int) SmartDashboard.getNumber(kShooterMeasurementWindowKey, 1));
        }

        SmartDashboard.putNumber(kShooterFlywheelVelocityKey, getVelocityRawUnits());
        SmartDashboard.putNumber(kShooterErrorRawUnitsKey, getClosedLoopErrorRawUnits());
        SmartDashboard.putNumber(kShooterErrorRPSKey, getClosedLoopErrorRevolutionsPerSec());
        SmartDashboard.putNumber(kShooterErrorInchesKey, getClosedLoopErrorInchesPerSec());
        SmartDashboard.putNumber(kShooterLimelightDistanceFeetKey, LimelightHelper.getDistanceToTargetFeet());
        SmartDashboard.putBoolean(kShooterAdjustableHoodStateKey, mAdjustableHoodSolenoid.get());
    }

    public ShooterConfig getConfig() {
        return mConfig;
    }

    public boolean checkSystem() {
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Checking Shooter Subsystem");

        final double kCurrentThres = 0.5;
        final double kRpmThres = 9000;

        mFlywheelMaster.set(TalonFXControlMode.PercentOutput, 6.0 / RobotController.getBatteryVoltage());
        Timer.delay(4.0);
        final double currentRightMaster = mFlywheelMaster.getStatorCurrent();
        final double rpmMaster = mFlywheelMaster.getSelectedSensorVelocity();
        mFlywheelMaster.set(TalonFXControlMode.PercentOutput, 0.0);

        Timer.delay(2.0);

        mFlywheelSlave.set(TalonFXControlMode.PercentOutput, 6.0 / RobotController.getBatteryVoltage());
        Timer.delay(4.0);
        final double currentRightSlave = mFlywheelSlave.getStatorCurrent();
        final double rpmFlywheelSlave = mFlywheelSlave.getSelectedSensorVelocity();
        mFlywheelSlave.set(TalonFXControlMode.PercentOutput, 0.0);

        Timer.delay(2.0);

        configureFlywheelControllers();

        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                ("Shooter Flywheel Master Current: " + currentRightMaster + " Shooter Flywheel Slave Current: "
                        + currentRightSlave));

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel Master Current Low!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel Slave Current Low!");
        }

        if (!UtilMethods.allWithinTolerance(Arrays.asList(
                currentRightMaster, currentRightSlave), currentRightMaster, 5.0)) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel Currents Different!");
        }

        if (rpmMaster < kRpmThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel Master RPM Low!");
        }

        if (rpmFlywheelSlave < kRpmThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel Slave RPM Low!");
        }

        if (!UtilMethods.allWithinTolerance(Arrays.asList(rpmMaster, rpmFlywheelSlave), rpmMaster, 1000)) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Shooter Flywheel RPMs Different!");
        }

        if (mAdjustableHoodSolenoid.getPCMSolenoidVoltageFault()) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Adjustable hood solenoid voltage fault!");
        }

        return !failure;
    }

}