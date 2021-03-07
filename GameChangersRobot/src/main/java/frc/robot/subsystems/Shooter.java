package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kFlywheelMasterID;
import static frc.robot.Constants.CANBusIDs.kFlywheelSlaveID;
import static frc.robot.Constants.PIDSlots.kShooterOpenLoopSlot;
import static frc.robot.Constants.PIDSlots.kShooterSpinningUpSlot;
import static frc.robot.Constants.Shooter.kVoltageSaturation;

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

//        TODO: Check the following two settings by running flywheels on manual joystick
//        mFlywheelMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
//        mFlywheelMaster.configVelocityMeasurementWindow(32);

        mFlywheelMaster.config_kF(kShooterSpinningUpSlot, 0.0498575917);
        mFlywheelMaster.config_kP(kShooterSpinningUpSlot, 0.23);
        mFlywheelMaster.config_kI(kShooterSpinningUpSlot, 0);
        mFlywheelMaster.config_kD(kShooterSpinningUpSlot, 0);

        mFlywheelMaster.config_kF(kShooterOpenLoopSlot, 0.0498575917);
        mFlywheelMaster.config_kP(kShooterOpenLoopSlot, 0);
        mFlywheelMaster.config_kI(kShooterOpenLoopSlot, 0);
        mFlywheelMaster.config_kD(kShooterOpenLoopSlot, 0);
    }

    public void configureForSpinningUp() {
        setProfileSlot(kShooterSpinningUpSlot);

        mFlywheelMaster.enableVoltageCompensation(false);
        mFlywheelSlave.enableVoltageCompensation(false);
    }

    public void configureForOpenLoop(double kFF) {
        setProfileSlot(kShooterOpenLoopSlot);

        mFlywheelMaster.configVoltageCompSaturation(kVoltageSaturation);
        mFlywheelMaster.enableVoltageCompensation(true);
        mFlywheelSlave.configVoltageCompSaturation(kVoltageSaturation);
        mFlywheelSlave.enableVoltageCompensation(false);

        mFlywheelMaster.config_kF(kShooterOpenLoopSlot, kFF);
    }

    public void setFF(double kFF) {
        mFlywheelMaster.config_kF(kShooterOpenLoopSlot, kFF);
    }

    public void setProfileSlot(int profileSlot) {
        mFlywheelMaster.selectProfileSlot(profileSlot, 0);
    }

    public void setOpenLoopDutyCycle(double targetDutyCycle) {
        mFlywheelMaster.set(ControlMode.PercentOutput, targetDutyCycle);
        mFlywheelSlave.set(ControlMode.Follower, kFlywheelMasterID);
    }

    public void setClosedLoopVelocityRawUnits(double targetVelocity) {
        mFlywheelMaster.set(ControlMode.Velocity, targetVelocity);
        mFlywheelSlave.set(ControlMode.Follower, kFlywheelMasterID);
    }

    public double getVelocityRawUnits() {
        return mFlywheelMaster.getSelectedSensorVelocity();
    }

    public double getClosedLoopErrorRawUnits() {
        return mFlywheelMaster.getClosedLoopError();
    }

    public double getEstimatedKf() {
        return ((mFlywheelMaster.getMotorOutputPercent() * 1023) / getVelocityRawUnits());
    }

    @Override
    public void periodic() {

    }

}