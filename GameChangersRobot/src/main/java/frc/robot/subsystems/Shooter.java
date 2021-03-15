package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.UtilMethods;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.PIDSlots.kShooterShootingSlot;
import static frc.robot.Constants.PIDSlots.kShooterSpinningUpSlot;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;

public class Shooter extends SubsystemBase {

    private final TalonFX mFlywheelMaster = new TalonFX(kFlywheelMasterID);
    private final TalonFX mFlywheelSlave = new TalonFX(kFlywheelSlaveID);

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

//        TODO: Check the following two settings by scheduling the ShooterMeasurementTuning command
//        mFlywheelMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
//        mFlywheelMaster.configVelocityMeasurementWindow(32);

        mFlywheelMaster.config_kF(kShooterSpinningUpSlot, 0.04934694);
        mFlywheelMaster.config_kP(kShooterSpinningUpSlot, 0.2);
        mFlywheelMaster.config_kI(kShooterSpinningUpSlot, 0.002);
        mFlywheelMaster.config_kD(kShooterSpinningUpSlot, 0);
        mFlywheelMaster.config_IntegralZone(kShooterSpinningUpSlot, 600);

        mFlywheelMaster.config_kF(kShooterShootingSlot, 0.04934694);
        mFlywheelMaster.config_kP(kShooterShootingSlot, 0.21);
        mFlywheelMaster.config_kI(kShooterShootingSlot, 0.002);
        mFlywheelMaster.config_kD(kShooterShootingSlot, 0);
        mFlywheelMaster.config_IntegralZone(kShooterShootingSlot, 600);

        // Voltage compensation
        mFlywheelMaster.configVoltageCompSaturation(10);
        mFlywheelMaster.enableVoltageCompensation(true);
        mFlywheelSlave.configVoltageCompSaturation(10);
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

    public double getEstimatedVelocityFromTarget() {
        // If the limelight does not see a target, we use the last known "ty" value since
        // LimelightHelper uses a MovingAverage to keep track of it at all times

        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();

        distanceFeet = UtilMethods.limitRange(distanceFeet, kAbsoluteShootingDistanceFloor,
                kAbsoluteShootingDistanceCeiling);

        if (kUseInterpolationMap) {
            InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map
                    = sCurrentRobot.getCurrentRobot().getShooterMap();

            InterpolatingDouble result = map.getInterpolated(new InterpolatingDouble(distanceFeet));

            if (result != null) {
                return result.value;
            } else {
                return map.getInterpolated(new InterpolatingDouble(kDefaultShootingDistanceFeet)).value;
            }
        } else {
            return sCurrentRobot.getCurrentRobot().getShooterPolynomial().predict(distanceFeet);
        }
    }

    @Override
    public void periodic() {
        LimelightHelper.updateData();

        if (kIsInTuningMode) {
            mFlywheelMaster.configVelocityMeasurementPeriod(
                    VelocityMeasPeriod.valueOf(SmartDashboard.getNumber(kShooterMeasurementPeriodKey, 1)));
            mFlywheelMaster.configVelocityMeasurementWindow(
                    (int)SmartDashboard.getNumber(kShooterMeasurementWindowKey, 1));
        }

        SmartDashboard.putNumber(kShooterFlywheelVelocityKey, getVelocityRawUnits());
        SmartDashboard.putNumber(kShooterErrorRawUnitsKey, getClosedLoopErrorRawUnits());
        SmartDashboard.putNumber(kShooterErrorRPSKey, getClosedLoopErrorRevolutionsPerSec());
        SmartDashboard.putNumber(kShooterErrorInchesKey, getClosedLoopErrorInchesPerSec());
        SmartDashboard.putNumber(kShooterLimelightDistanceFeetKey, LimelightHelper.getDistanceToTargetFeet());
    }

}