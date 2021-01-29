package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robots.RobotIdentification;
import frc.robot.robots.WaltRobot;

import static frc.robot.Constants.DrivetrainPIDSlots.VELOCITY_PID_SLOT;
import static frc.robot.Constants.DrivetrainPIDSlots.VOLTAGE_PID_SLOT;
import static frc.robot.Constants.Hardware.*;
import static frc.robot.Robot.currentRobot;

public class Drivetrain extends SubsystemBase {

    public static CANSparkMax mLeftWheelsMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsMaster = new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mLeftWheelsSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    public Drivetrain() {
        setupMotors();
        reset();
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
        mRightWheelSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mLeftWheelsSlave.follow(mLeftWheelsMaster);
        mRightWheelSlave.follow(mRightWheelsMaster);

        mLeftWheelsMaster.setOpenLoopRampRate(0);
        mLeftWheelsSlave.setOpenLoopRampRate(0);
        mRightWheelsMaster.setOpenLoopRampRate(0);
        mRightWheelSlave.setOpenLoopRampRate(0);

        mLeftWheelsMaster.setSmartCurrentLimit(80);
        mLeftWheelsSlave.setSmartCurrentLimit(80);
        mRightWheelsMaster.setSmartCurrentLimit(80);
        mRightWheelSlave.setSmartCurrentLimit(80);

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getP(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getI(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getD(), VELOCITY_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getP(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getI(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainVelocityPID().getD(), VELOCITY_PID_SLOT);

//        leftWheelsMaster.burnFlash();
//        leftWheelsSlave.burnFlash();
//        rightWheelsSlave.burnFlash();
//        rightWheelsMaster.burnFlash();
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

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    private ProfiledPIDController mDriveStraightPowerController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(5, 10));

    public ProfiledPIDController getmDriveStraightPowerController() {
        return mDriveStraightPowerController;
    }

    public ProfiledPIDController getmDriveStraightHeadingPIDController() {
        return mDriveStraightHeadingPIDController;
    }

    private ProfiledPIDController mDriveStraightHeadingPIDController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));
}

