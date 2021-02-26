package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kFlywheelMasterID;
import static frc.robot.Constants.CANBusIDs.kFlywheelSlaveID;

public class Shooter extends SubsystemBase {

    private final TalonFX mFlywheelMaster = new TalonFX(kFlywheelMasterID);
    private final TalonFX mFlywheelSlave = new TalonFX(kFlywheelSlaveID);

    public Shooter() {
        setupFlywheelControllers();
    }

    private void setupFlywheelControllers() {
        mFlywheelMaster.selectProfileSlot(0, 0);

        mFlywheelMaster.setNeutralMode(NeutralMode.Coast);
        mFlywheelSlave.setNeutralMode(NeutralMode.Coast);

        mFlywheelMaster.setInverted(true);
        mFlywheelSlave.setInverted(false);

        mFlywheelMaster.config_kF(0, .0498575917);
        mFlywheelMaster.config_kP(0, 0.23);
        mFlywheelMaster.config_kI(0, 0);
        mFlywheelMaster.config_kD(0, 0);

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

    public void setClosedLoopVelocity(double targetVelocity) {
        mFlywheelMaster.set(ControlMode.Velocity, targetVelocity);
    }

    public double getVelocity() {
        return mFlywheelMaster.getSensorCollection().getIntegratedSensorVelocity();
    }

    @Override
    public void periodic() {

    }

}