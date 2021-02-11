package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.OI.*;

import frc.robot.utils.MovingAverage;

public class Shooter extends SubsystemBase {
    private final TalonFX flywheelMaster = new TalonFX(kFlyMaster);
    private final TalonFX flywheelSlave = new TalonFX(kFlySlave);

    private double minShootingDistance = 9;
    private double maxShootingDistance = 25;

    private double minDistanceRPM = 12800;
    private double maxDistanceRPM = 13000;

    public boolean isReadyToShoot = false;

    private MovingAverage movingAverage = new MovingAverage(5);

    public Shooter() {
        setupFlywheelControllers();
    }

    private void setupFlywheelControllers() {
        flywheelMaster.selectProfileSlot(0, 0);

        flywheelMaster.setNeutralMode(NeutralMode.Coast);
        flywheelSlave.setNeutralMode(NeutralMode.Coast);

        flywheelMaster.setInverted(true);
        flywheelSlave.setInverted(false);

        flywheelMaster.config_kF(0, 0.056);
        flywheelMaster.config_kP(0, 0.013);
        flywheelMaster.config_kD(0, 0);

        // Voltage compensation
        flywheelMaster.configVoltageCompSaturation(10);
        flywheelMaster.enableVoltageCompensation(true);
        flywheelSlave.configVoltageCompSaturation(10);
        flywheelSlave.enableVoltageCompensation(false);
    }

    public void setProfileSlot(int profileSlot) {
        flywheelMaster.selectProfileSlot(profileSlot, 0);
    }

    public void setOpenLoopDutyCycles(double targetDutyCycles) {
        mFlyWheelMaster.set(ControlMode.Velocity, targetDutyCycles);
    }

    public void setClosedLoopVelocity(double targetVelocity) {
        mFlyWheelMaster.set(ControlMode.Velocity, targetVelocity);
    }

    public double getVelocity() {
        return mFlyWheelMaster.getSensorCollection().getIntegratedSensorVelocity();
    }

    @Override
    public void periodic() {
        movingAverage.addData(getVelocity());
    }

    public double getAverageClosedLoopVelocity(){
        return movingAverage.getMean();
    }

}


