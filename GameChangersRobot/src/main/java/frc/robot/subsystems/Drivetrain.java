package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.auton.LiveDashboardHelper;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.PIDSlots.kDrivetrainVelocitySlot;
import static frc.robot.Constants.PIDSlots.kDrivetrainVoltageSlot;
import static frc.robot.Robot.sCurrentRobot;
import static frc.robot.Robot.sDrivetrain;

public class Drivetrain extends SubsystemBase {

    public static CANSparkMax mLeftWheelsMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsMaster = new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mLeftWheelsSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final LinearSystem<N2, N2, N2> driveModel = LinearSystemId.identifyDrivetrainSystem(
            sCurrentRobot.getCurrentRobot().getDrivetrainFeedforward().kv,
            0.5,
            1.0, 1.0
    );
    private final KalmanFilter<N2, N2, N2> driveObserver = new KalmanFilter<>(
            Nat.N2(), Nat.N2(),
            driveModel,
            VecBuilder.fill(3.0, 3.0), // Standard deviations of drivetrain model
            VecBuilder.fill(0.01, 0.01), // Standard deviations of encoder velocities
            0.02
    );
    private final LinearQuadraticRegulator<N2, N2, N2> driveLQRController = new LinearQuadraticRegulator<>(
            driveModel,
            VecBuilder.fill(8.0, 8.0), // qelms. Velocity error tolerance, in radians per second
            VecBuilder.fill(12.0, 12.0), // relms. Control effort (voltage) tolerance
            0.02
    );
    private final LinearSystemLoop<N2, N2, N2> driveControlLoop = new LinearSystemLoop<>(
            driveModel,
            driveLQRController,
            driveObserver,
            12.0, 0.02
    );

    private final DifferentialDriveKinematics mDriveKinematics = new DifferentialDriveKinematics(sCurrentRobot.getCurrentRobot().getTrackWidth());
    private final DifferentialDriveOdometry mDriveOdometry = new DifferentialDriveOdometry(getHeading());
    private final RamseteController mRamseteController = new RamseteController();

    private Pose2d mCurrentPose = new Pose2d();

    public Drivetrain() {
        setupMotorsTeleop();
        reset();
    }

    public DifferentialDriveKinematics getDriveKinematics() {
        return mDriveKinematics;
    }

    public DifferentialDriveOdometry getDriveOdometry() {
        return mDriveOdometry;
    }

    public RamseteController getRamseteController() {
        return mRamseteController;
    }

    public Pose2d getCurrentPose() {
        return mCurrentPose;
    }

    public LinearSystemLoop<N2, N2, N2> getDriveControlLoop() {
        return driveControlLoop;
    }

    public void setupMotorsTeleop() {
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

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsSlave.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());

        mLeftWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getP(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getI(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getD(), kDrivetrainVoltageSlot);

        mRightWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getP(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getI(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getD(), kDrivetrainVoltageSlot);

        mLeftWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getI(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getD(), kDrivetrainVelocitySlot);

        mRightWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getI(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getD(), kDrivetrainVelocitySlot);

        mLeftWheelsMaster.burnFlash();
        mLeftWheelsSlave.burnFlash();
        mRightWheelsSlave.burnFlash();
        mRightWheelsMaster.burnFlash();
    }

    public void setupMotorsAuton() {
        mLeftWheelsMaster.setInverted(true);

        mLeftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
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

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsSlave.getEncoder().setPositionConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainPositionFactor());

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(sCurrentRobot.getCurrentRobot().getDrivetrainVelocityFactor());

        mLeftWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getP(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getI(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getD(), kDrivetrainVoltageSlot);

        mRightWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getP(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getI(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getD(), kDrivetrainVoltageSlot);

        mLeftWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getI(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getD(), kDrivetrainVelocitySlot);

        mRightWheelsMaster.getPIDController().setP(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setI(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getI(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setD(sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getD(), kDrivetrainVelocitySlot);

        mLeftWheelsMaster.burnFlash();
        mLeftWheelsSlave.burnFlash();
        mRightWheelsSlave.burnFlash();
        mRightWheelsMaster.burnFlash();
    }

    @Override
    public void periodic() {
//        mLeftWheelsMaster.getPIDController().setP(SmartDashboard.getNumber(kLeftVelocityPKey,
//                sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP()), kDrivetrainVelocitySlot);
//
//        mRightWheelsMaster.getPIDController().setP(SmartDashboard.getNumber(kRightVelocityPKey,
//                sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP()), kDrivetrainVelocitySlot);

        updateRobotPose();
        SmartDashboard.putNumber("Angular Rate", getAngularVelocity());
        SmartDashboard.putNumber("Angle", getHeading().getDegrees());
        SmartDashboard.putNumber("Left neo encoder velocity", getLeftVelocity());
        SmartDashboard.putNumber("right neo encoder velocity", getRightVelocity());
        SmartDashboard.putNumber("Left neo encoder distance", getLeftMetersDisplacement());
        SmartDashboard.putNumber("right neo encoder distance", getRightMetersDisplacement());

        LiveDashboardHelper.putRobotData(sDrivetrain.getCurrentPose());
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

    public void setVoltages(double leftVoltage, double rightVoltage) {
        mLeftWheelsMaster.setVoltage(leftVoltage);
        mRightWheelsMaster.setVoltage(rightVoltage);
    }

    public void setVelocities(double leftVelocity, double leftFeedForward, double rightVelocity, double rightFeedForward) {
        mLeftWheelsMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity, kDrivetrainVelocitySlot, leftFeedForward, CANPIDController.ArbFFUnits.kVoltage);
        mRightWheelsMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity, kDrivetrainVelocitySlot, rightFeedForward, CANPIDController.ArbFFUnits.kVoltage);
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

    public void resetPose(Pose2d startingPose) {
        resetEncoders();
        mDriveOdometry.resetPosition(startingPose, getHeading());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                getLeftVelocity(),
                getRightVelocity()
        );
    }

    public double getLeftMetersDisplacement() {
        return mLeftWheelsMaster.getEncoder().getPosition();
    }

    public double getRightMetersDisplacement() {
        return mRightWheelsMaster.getEncoder().getPosition();
    }

    public double getLeftVelocity() {
        return mLeftWheelsMaster.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return mRightWheelsMaster.getEncoder().getVelocity();
    }

    private void updateRobotPose() {
        mCurrentPose = mDriveOdometry.update(getHeading(), mLeftWheelsMaster.getEncoder().getPosition(), mRightWheelsMaster.getEncoder().getPosition());
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    public double getAngularVelocity() {
        return -ahrs.getRate();
    }

}