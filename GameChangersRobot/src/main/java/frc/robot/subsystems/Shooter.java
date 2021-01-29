package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.defaultShooterRPM;
import static frc.robot.Constants.Turret.TURRET_ENCODER_PORT_1;
import static frc.robot.Constants.Turret.TURRET_ENCODER_PORT_2;
import static frc.robot.Constants.Turret.TURRET_ROTATIONS_PER_TICK;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.OI.mFlyWheelMaster;

public class Shooter extends SubsystemBase {
    private final TalonFX flywheelMaster = new TalonFX(9);
    private final TalonFX flywheelSlave = new TalonFX(10);

    private double minShootingDistance = 9;
    private double maxShootingDistance = 25;

    private double minDistanceRPM = 12800;
    private double maxDistanceRPM = 13000;

    public boolean isReadyToShoot = false;

    public Shooter(double targetVelocity) {


        //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20, 10);

    }
    private void setOpenLoopDutyCycles(double targetDutyCycles) {
        mFlyWheelMaster.set(ControlMode.Velocity, targetDutyCycles);
    }

    private void setClosedLoopVelocity (double targetVelocity) {
        mFlyWheelMaster.set(ControlMode.Velocity, targetVelocity);
    }

    private double getVelocity(){
        return mFlyWheelMaster.getSelectedSensorVelocity();
    }


    private double getFlyWheelSpeed(){
        return mFlyWheelMaster.getSensorCollection().getIntegratedSensorVelocity();
    }

}