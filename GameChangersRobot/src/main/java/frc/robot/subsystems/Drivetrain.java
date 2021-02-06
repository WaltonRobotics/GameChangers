package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;

import static frc.robot.Constants.DrivetrainPIDSlots.VELOCITY_PID_SLOT;
import static frc.robot.Constants.DrivetrainPIDSlots.VOLTAGE_PID_SLOT;
import static frc.robot.Constants.Hardware.*;
import static frc.robot.Robot.currentRobot;

public class Drivetrain extends SubsystemBase {

    public static CANSparkMax mLeftWheelsMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsMaster = new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mLeftWheelsSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax mRightWheelsSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final LinearSystem<N2, N2, N2> driveModel = LinearSystemId.identifyDrivetrainSystem(
            currentRobot.getCurrentRobot().getDrivetrainFeedforward().kv,
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

    private final DifferentialDriveKinematics mDriveKinematics = new DifferentialDriveKinematics(currentRobot.getCurrentRobot().getTrackWidth());
    private final DifferentialDriveOdometry mDriveOdometry = new DifferentialDriveOdometry(getHeading());
    private final RamseteController mRamseteController = new RamseteController();

    private Pose2d mCurrentPose = new Pose2d();

    private final ProfiledPIDController mDriveStraightPowerController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(5, 10));
    private final ProfiledPIDController mDriveStraightHeadingPIDController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    public Drivetrain() {
        setupMotors();
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

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getI(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getD(), VELOCITY_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getI(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getD(), VELOCITY_PID_SLOT);

        mLeftWheelsMaster.burnFlash();
        mLeftWheelsSlave.burnFlash();
        mRightWheelsSlave.burnFlash();
        mRightWheelsMaster.burnFlash();
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

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());
        mRightWheelsSlave.getEncoder().setPositionConversionFactor(currentRobot.getCurrentRobot().getDrivetrainPositionFactor());

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());
        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(currentRobot.getCurrentRobot().getDrivetrainVelocityFactor());

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getP(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getI(), VOLTAGE_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVoltagePID().getD(), VOLTAGE_PID_SLOT);

        mLeftWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getI(), VELOCITY_PID_SLOT);
        mLeftWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getD(), VELOCITY_PID_SLOT);

        mRightWheelsMaster.getPIDController().setP(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setI(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getI(), VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setD(currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getD(), VELOCITY_PID_SLOT);

        mLeftWheelsMaster.burnFlash();
        mLeftWheelsSlave.burnFlash();
        mRightWheelsSlave.burnFlash();
        mRightWheelsMaster.burnFlash();
    }

    @Override
    public void periodic() {
        //mLeftWheelsMaster.getPIDController().setP(SmartDashboard.getNumber("Left Velocity P", 1.25), VELOCITY_PID_SLOT);
        //mRightWheelsMaster.getPIDController().setP(SmartDashboard.getNumber("Right Velocity P", 1.25), VELOCITY_PID_SLOT);

        updateRobotPose();
        SmartDashboard.putNumber("Angular Rate", getAngularVelocity());
        SmartDashboard.putNumber("Angle", getHeading().getDegrees());
        SmartDashboard.putNumber("Left neo encoder velocity", getLeftVelocity());
        SmartDashboard.putNumber("right neo encoder velocity", getRightVelocity());
        SmartDashboard.putNumber("Left neo encoder distance", getLeftMetersDisplacement());
        SmartDashboard.putNumber("right neo encoder distance", getRightMetersDisplacement());

        var translation = mDriveOdometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
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
        mLeftWheelsMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity, VELOCITY_PID_SLOT, leftFeedForward, CANPIDController.ArbFFUnits.kVoltage);
        //mLeftWheelsMaster.getPIDController().setP(0, VELOCITY_PID_SLOT);
        mRightWheelsMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity, VELOCITY_PID_SLOT, rightFeedForward, CANPIDController.ArbFFUnits.kVoltage);
        //mRightWheelsMaster.getPIDController().setP(0, VELOCITY_PID_SLOT);
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

    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

    public void resetOdometry(Pose2d startingPose) {
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

    public ProfiledPIDController getmDriveStraightPowerController() {
        return mDriveStraightPowerController;
    }

    public ProfiledPIDController getmDriveStraightHeadingPIDController() {
        return mDriveStraightHeadingPIDController;
    }
}