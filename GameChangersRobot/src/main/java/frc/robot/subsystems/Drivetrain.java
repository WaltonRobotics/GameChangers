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

    private void resetEncoders() {
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
//
//        mLeftWheelsMaster.burnFlash();
//        mLeftWheelsSlave.burnFlash();
//        mRightWheelsSlave.burnFlash();
//        mRightWheelsMaster.burnFlash();
    }

    private ProfiledPIDController mDriveStraightPowerController = new ProfiledPIDController(80, 0, 0,
            new TrapezoidProfile.Constraints(0.5,0.25));

    public ProfiledPIDController getmDriveStraightPowerController() {
        return mDriveStraightPowerController;
    }

    public ProfiledPIDController getmDriveStraightHeadingPIDController() {
        return mDriveStraightHeadingPIDController;
    }

    private ProfiledPIDController mDriveStraightHeadingPIDController = new ProfiledPIDController(20, 0, 0,
            new TrapezoidProfile.Constraints(60, 30));
}