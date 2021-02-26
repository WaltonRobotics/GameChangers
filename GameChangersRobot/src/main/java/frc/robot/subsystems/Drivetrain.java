package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SmartDashboardKeys.*;
import java.util.Arrays;
import frc.robot.utils.*;
import jdk.jshell.execution.Util;
import frc.robot.Constants;
import java.util.Optional;

import static frc.robot.Constants.Hardware.*;

public class Drivetrain extends SubsystemBase {
    public static CANSparkMax mLeftWheelsMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsMaster = new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mLeftWheelsSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    @Override
    public void periodic() {
        SmartDashboard.putNumber(kleftEncodervalue, leftMetersTravelled());
        SmartDashboard.putNumber(krightEncoderValue, rightMetersTravelled());
    }

    public Drivetrain(){
        setupMotors();
        reset();
    }

    public double leftMetersTravelled(){
        return mLeftWheelsMaster.getEncoder().getPosition();
    }

    public double rightMetersTravelled(){
        return mRightWheelsMaster.getEncoder().getPosition();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    public void reset() {
        resetEncoders();
        resetHeading();
    }

    public void resetEncoders() {
        mLeftWheelsMaster.getEncoder().setPosition(0);
        mRightWheelsMaster.getEncoder().setPosition(0);
    }

    public void resetHeading() {
        ahrs.zeroYaw();
    }

    public void setArcadeSpeeds(double xSpeed, double zRotation) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);

        double leftMotorOutput;
        double rightMotorOutput;

        xSpeed = Math
                .max(-1.0 + Math.abs(zRotation),
                        Math.min(1.0 - Math.abs(zRotation), xSpeed));

        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = xSpeed - zRotation;

        setDutyCycles(leftMotorOutput, rightMotorOutput);
    }

    public void setDutyCycles(double leftDutyCycle, double rightDutyCycle) {
        mLeftWheelsMaster.set(leftDutyCycle);
        mRightWheelsMaster.set(rightDutyCycle);
    }

    public void setupMotors() {
        //        leftWheelsMaster.restoreFactoryDefaults();
        //        leftWheelsSlave.restoreFactoryDefaults();
        //        rightWheelsMaster.restoreFactoryDefaults();
        //        rightWheelsSlave.restoreFactoryDefaults();
        mLeftWheelsMaster.setInverted(true);

        mLeftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mLeftWheelsSlave.follow(mLeftWheelsMaster);
        mRightWheelsSlave.follow(mRightWheelsMaster);

        mLeftWheelsMaster.setOpenLoopRampRate(0);
        mLeftWheelsSlave.setOpenLoopRampRate(0);
        mRightWheelsMaster.setOpenLoopRampRate(0);
        mRightWheelsSlave.setOpenLoopRampRate(0);

        mLeftWheelsMaster.setSmartCurrentLimit(80);
        mLeftWheelsSlave.setSmartCurrentLimit(80);
        mRightWheelsMaster.setSmartCurrentLimit(80);
        mRightWheelsSlave.setSmartCurrentLimit(80);

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(1 / 64.125);
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(1 / 64.125);

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor((1 / 64.125) / 60);
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor((1 / 64.125) / 60);

        mLeftWheelsMaster.burnFlash();
        mLeftWheelsSlave.burnFlash();
        mRightWheelsSlave.burnFlash();
        mRightWheelsMaster.burnFlash();

//        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
//        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
//        mRightWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
//        mRightWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
//
//        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
//        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
//        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
//        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
//
//        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getP(), VOLTAGE_PID_SLOT);
//        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getI(), VOLTAGE_PID_SLOT);
//        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getD(), VOLTAGE_PID_SLOT);
//
//        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getP(), VOLTAGE_PID_SLOT);
//        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getI(), VOLTAGE_PID_SLOT);
//        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getD(), VOLTAGE_PID_SLOT);
//
//        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP(), VELOCITY_PID_SLOT);
//        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getI(), VELOCITY_PID_SLOT);
//        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getD(), VELOCITY_PID_SLOT);
//
//        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP(), VELOCITY_PID_SLOT);
//        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getI(), VELOCITY_PID_SLOT);
//        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getD(), VELOCITY_PID_SLOT);
    }
    public boolean checkSystem() {
        System.out.println("Testing DRIVE.---------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 300;

        mRightWheelsMaster.changeControlMode(CANSparkMax.TalonControlMode.Voltage);
        mRightWheelsSlave.changeControlMode(CANSparkMax.TalonControlMode.Voltage);
        mLeftWheelsMaster.changeControlMode(CANSparkMax.TalonControlMode.Voltage);
        mLeftWheelsSlave.changeControlMode(CANSparkMax.TalonControlMode.Voltage);

        mRightWheelsMaster.set(0.0);
        mRightWheelsSlave.set(0.0);
        mLeftWheelsMaster.set(0.0);
        mLeftWheelsSlave.set(0.0);

        mRightWheelsMaster.set(-6.0f);
        Timer.delay(4.0);
        final double currentRightMaster = mRightWheelsMaster.getOutputCurrent();
        final double rpmRightWheelsMaster = mRightWheelsMaster.getSpeed();
        mRightWheelsMaster.set(0.0f);

        Timer.delay(2.0);

        mRightWheelsSlave.set(-6.0f);
        Timer.delay(4.0);
        final double currentRightSlave = mRightWheelsSlave.getOutputCurrent();
        final double rpmRightWheelsSlave = mRightWheelsMaster.getSpeed();
        mRightWheelsSlave.set(0.0f);

        Timer.delay(2.0);

        mLeftWheelsMaster.set(6.0f);
        Timer.delay(4.0);
        final double currentLeftMaster = mLeftWheelsMaster.getOutputCurrent();
        final double rpmLeftWheelsMaster = mLeftWheelsMaster.getspeed();
        mLeftWheelsMaster.set(0.0f);

        Timer.delay(2.0);

        mLeftWheelsSlave.set(6.0f);
        Timer.delay(4.0);
        final double currentLeftSlave = mLeftWheelsSlave.getOutputCurrent();
        final double rpmLeftWheelsSlave = mLeftWheelsMaster.getSpeed();
        mLeftWheelsSlave.set(0.0);

        mRightWheelsMaster.changeControlMode(CANSparkMax.TalonControlMode.PercentVbus);
        mLeftWheelsMaster.changeControlMode(CANSparkMax.TalonControlMode.PercentVbus);

        mRightWheelsSlave.changeControlMode(CANSparkMax.TalonControlMode.Follower);
        mRightWheelsSlave.set(Constants.kLeftMaster);

        mLeftWheelsSlave.changeControlMode(CANSparkMax.TalonControlMode.Follower);
        mLeftWheelsSlave.set(Constants.kLeftDriveMasterId);

        System.out.println("Drive Right Master Current: " + currentRightMaster + " Drive Right Slave Current: "
                + currentRightSlave);
        System.out.println(
                "Drive Left Master Current: " + currentLeftMaster + " Drive Left Slave Current: " + currentLeftSlave);
        System.out.println("Drive RPM RMaster: " + rpmRightWheelsMaster + " RSlave: " + rpmRightWheelsSlave + " LMaster: "
                + rpmLeftWheelsMaster + " LSlave: " + rpmLeftWheelsSlave);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave Current Low !!!!!!!!!!");
        }

        if (currentLeftMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave Current Low !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave), currentRightMaster,
                5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Currents Different !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentLeftMaster, currentLeftSlave), currentLeftSlave,
                5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Currents Different !!!!!!!!!!!!!");
        }

        if (rpmRightWheelsMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmRightWheelsSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Right Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftWheelsMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Master RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftWheelsSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Drive Left Slave RPM Low !!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmRightWheelsMaster, rpmRightWheelsSlave, rpmLeftWheelsMaster, rpmLeftWheelsSlave),
                rpmRightWheelsMaster, 250)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }

    private ProfiledPIDController turnPIDController = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    public ProfiledPIDController getTurnPIDController() {
        return turnPIDController;
    }

    private ProfiledPIDController mDriveStraightPowerController = new ProfiledPIDController(0.8, 0, 0,
            new TrapezoidProfile.Constraints(1.5,1.5));

    public ProfiledPIDController getmDriveStraightPowerController() {
        return mDriveStraightPowerController;
    }

    public ProfiledPIDController getmDriveStraightHeadingPIDController() {
        return mDriveStraightHeadingPIDController;
    }

    private ProfiledPIDController mDriveStraightHeadingPIDController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(60, 30));
}