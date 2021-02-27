package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import frc.robot.utils.Util;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.OI.*;

import frc.robot.utils.SimpleMovingAverage;


public class Shooter extends SubsystemBase {
    private final TalonFX flywheelMaster = new TalonFX(kFlyMaster);
    private final TalonFX flywheelSlave = new TalonFX(kFlySlave);

    private double minShootingDistance = 9;
    private double maxShootingDistance = 25;

    private double minDistanceRPM = 12800;
    private double maxDistanceRPM = 13000;

    public boolean isReadyToShoot = false;

    private SimpleMovingAverage movingAverage = new SimpleMovingAverage(5);

    public Shooter() {
        setupFlywheelControllers();
    }

    private void setupFlywheelControllers() {
        flywheelMaster.selectProfileSlot(0, 0);

        flywheelMaster.setNeutralMode(NeutralMode.Coast);
        flywheelSlave.setNeutralMode(NeutralMode.Coast);

        flywheelMaster.setInverted(true);
        flywheelSlave.setInverted(false);

        flywheelMaster.config_kF(0, .0498575917);
        flywheelMaster.config_kP(0, 0.23);
        flywheelMaster.config_kI(0, 0);
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

    public boolean checkSystem() {
        System.out.println("Testing SHOOTER.----------------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 1200;

        flywheelMaster.set(TalonFXControlMode.PercentOutput, 6.0 / getBatteryVoltage());
        Timer.delay(4.0);
        final double currentRightMaster = flywheelMaster.getStatorCurrent();
        final double rpmMaster = flywheelMaster.getSelectedSensorVelocity();
        flywheelMaster.set(TalonFXControlMode.PercentOutput, 0.0 / getBatteryVoltage());

        Timer.delay(2.0);

        flywheelSlave.set(TalonFXControlMode.PercentOutput, 6.0 / getBatteryVoltage());
        Timer.delay(4.0);
        final double currentRightSlave = flywheelSlave.getOutputCurrent();
        final double rpflywheelSlave = flywheelMaster.getSelectedSensorVelocity();
        flywheelSlave.set(TalonFXControlMode.PercentOutput, 0.0 / getBatteryVoltage());

        Timer.delay(2.0);

//        mLeftSlave1.set(-6.0f);

        Timer.delay(2.0);

        setupFlywheelControllers();

        System.out.println("Shooter Fly Master Current: " + currentRightMaster + " Shooter Fly Slave Current: "
                + currentRightSlave);
//        System.out.println("Shooter RPM Master: " + rpmMaster + " RPM Right slave: " + rpflywheelSlave
//                + " RPM Left Slave 1: " + rpmLeftSlave1 + " RPM Left Slave 2: " + rpmLeftSlave2);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Fly Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Fly Slave Current Low !!!!!!!!!!");
        }


        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave), currentRightMaster, 5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter currents different !!!!!!!!!!!!!!!!!");
        }

        if (rpmMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Master RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (rpflywheelSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Slave RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmMaster, rpflywheelSlave), rpmMaster, 250)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter RPMs different !!!!!!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }

}