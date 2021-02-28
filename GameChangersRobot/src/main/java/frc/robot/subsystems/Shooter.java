package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kFlywheelMasterID;
import static frc.robot.Constants.CANBusIDs.kFlywheelSlaveID;

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

        // TODO: Check the following two settings by running flywheels on manual joystick
        mFlywheelMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
        mFlywheelMaster.configVelocityMeasurementWindow(32);

        mFlywheelMaster.config_kF(0, 0.0498575917);
        mFlywheelMaster.config_kP(0, 0.23);
        mFlywheelMaster.config_kI(0, 0);
        mFlywheelMaster.config_kD(0, 0);

        mFlywheelMaster.config_kF(1, 0.0498575917);
        mFlywheelMaster.config_kP(1, 0.23);
        mFlywheelMaster.config_kI(1, 0);
        mFlywheelMaster.config_kD(1, 0);

        // Voltage compensation
        mFlywheelMaster.configVoltageCompSaturation(10);
        mFlywheelMaster.enableVoltageCompensation(true);
        mFlywheelSlave.configVoltageCompSaturation(10);
        mFlywheelSlave.enableVoltageCompensation(false);
    }

    public void setProfileSlot(int profileSlot) {
        mFlywheelMaster.selectProfileSlot(profileSlot, 0);
    }

    public void setOpenLoopDutyCycles(double targetDutyCycles) {
        mFlywheelMaster.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    public void setClosedLoopVelocityRawUnits(double targetVelocity) {
        mFlywheelMaster.set(ControlMode.Velocity, targetVelocity);
    }

    public double getVelocityRawUnits() {
        return mFlywheelMaster.getSensorCollection().getIntegratedSensorVelocity();
    }

    public double getClosedLoopErrorRawUnits() {
        return mFlywheelMaster.getClosedLoopError();
    }

    @Override
    public void periodic() {

    }

}