package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.*;
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
import frc.robot.config.DrivetrainConfig;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.ContextFlags.kIsInCompetition;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.PIDSlots.kDrivetrainVelocitySlot;
import static frc.robot.Constants.PIDSlots.kDrivetrainVoltageSlot;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;

public class Drivetrain extends SubsystemBase {

    private final DrivetrainConfig mConfig = sCurrentRobot.getCurrentRobot().getDrivetrainConfig();

    private final CANSparkMax mLeftWheelsMaster = new CANSparkMax(kDrivetrainLeftMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax mRightWheelsMaster = new CANSparkMax(kDrivetrainRightMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax mLeftWheelsSlave = new CANSparkMax(kDrivetrainLeftSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax mRightWheelsSlave = new CANSparkMax(kDrivetrainRightSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AHRS mAhrs = new AHRS(SPI.Port.kMXP);

    private final SimpleMotorFeedforward mFeedforward = mConfig.feedforward;
    private final DifferentialDriveKinematics mDriveKinematics = new DifferentialDriveKinematics(
            mConfig.kTrackWidthMeters
    );
    private final DifferentialDriveOdometry mDriveOdometry = new DifferentialDriveOdometry(getHeading());
    private final RamseteController mRamseteController = new RamseteController();

    private final LinearSystem<N2, N2, N2> mDriveModel = LinearSystemId.identifyDrivetrainSystem(
            mFeedforward.kv,
            mFeedforward.ka,
            1.0, 1.0
    );
    private final KalmanFilter<N2, N2, N2> mDriveObserver = new KalmanFilter<>(
            Nat.N2(), Nat.N2(),
            mDriveModel,
            VecBuilder.fill(3.0, 3.0), // Standard deviations of drivetrain model
            VecBuilder.fill(0.01, 0.01), // Standard deviations of encoder velocities
            0.02
    );
    private final LinearQuadraticRegulator<N2, N2, N2> mDriveLQRController = new LinearQuadraticRegulator<>(
            mDriveModel,
            VecBuilder.fill(8.0, 8.0), // qelms. Velocity error tolerance, in radians per second
            VecBuilder.fill(12.0, 12.0), // relms. Control effort (voltage) tolerance
            0.02
    );
    private final LinearSystemLoop<N2, N2, N2> mDriveControlLoop = new LinearSystemLoop<>(
            mDriveModel,
            mDriveLQRController,
            mDriveObserver,
            12.0, 0.02
    );

    private final PIDController mLeftVoltagePID = mConfig.leftVoltagePID;
    private final PIDController mRightVoltagePID = mConfig.rightVoltagePID;
    private final PIDController mLeftVelocityPID = mConfig.leftVelocityPID;
    private final PIDController mRightVelocityPID = mConfig.rightVelocityPID;

    private final ProfiledPIDController mTurnProfiledPID = mConfig.turnProfiledPID;

    private final ProfiledPIDController mDriveStraightPowerProfiledPID = mConfig.driveStraightProfiledPowerPID;
    private final ProfiledPIDController mDriveStraightHeadingProfiledPID = mConfig.driveStraightProfiledHeadingPID;

    private Pose2d mCurrentPose = new Pose2d();

    public Drivetrain() {
        configureControllersTeleop();
        reset();
    }

    public void configureControllersTeleop() {
        configureControllersExcludingIdleMode();

        mLeftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

        handleBurnToFlash();
    }

    public void configureControllersAuton() {
        configureControllersExcludingIdleMode();

        mLeftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

        handleBurnToFlash();
    }

    private void configureControllersExcludingIdleMode() {
        mLeftWheelsMaster.restoreFactoryDefaults();
        mLeftWheelsSlave.restoreFactoryDefaults();
        mRightWheelsMaster.restoreFactoryDefaults();
        mRightWheelsSlave.restoreFactoryDefaults();

        mLeftWheelsMaster.setInverted(true);

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

        mLeftWheelsMaster.getEncoder().setPositionConversionFactor(mConfig.kPositionFactor);
        mLeftWheelsSlave.getEncoder().setPositionConversionFactor(mConfig.kPositionFactor);
        mRightWheelsMaster.getEncoder().setPositionConversionFactor(mConfig.kPositionFactor);
        mRightWheelsSlave.getEncoder().setPositionConversionFactor(mConfig.kPositionFactor);

        mLeftWheelsMaster.getEncoder().setVelocityConversionFactor(mConfig.kVelocityFactor);
        mLeftWheelsSlave.getEncoder().setVelocityConversionFactor(mConfig.kVelocityFactor);
        mRightWheelsMaster.getEncoder().setVelocityConversionFactor(mConfig.kVelocityFactor);
        mRightWheelsSlave.getEncoder().setVelocityConversionFactor(mConfig.kVelocityFactor);

        mLeftWheelsMaster.getPIDController().setP(mLeftVoltagePID.getP(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setI(mLeftVoltagePID.getI(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setD(mLeftVoltagePID.getD(), kDrivetrainVoltageSlot);
        mLeftWheelsMaster.getPIDController().setOutputRange(-1, 1, kDrivetrainVoltageSlot);

        mRightWheelsMaster.getPIDController().setP(mRightVoltagePID.getP(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setI(mRightVoltagePID.getI(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setD(mRightVoltagePID.getD(), kDrivetrainVoltageSlot);
        mRightWheelsMaster.getPIDController().setOutputRange(-1, 1, kDrivetrainVoltageSlot);

        mLeftWheelsMaster.getPIDController().setP(mLeftVelocityPID.getP(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setI(mLeftVelocityPID.getI(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setD(mLeftVelocityPID.getD(), kDrivetrainVelocitySlot);
        mLeftWheelsMaster.getPIDController().setOutputRange(-1, 1, kDrivetrainVelocitySlot);

        mRightWheelsMaster.getPIDController().setP(mRightVelocityPID.getP(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setI(mRightVelocityPID.getI(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setD(mRightVelocityPID.getD(), kDrivetrainVelocitySlot);
        mRightWheelsMaster.getPIDController().setOutputRange(-1, 1, kDrivetrainVelocitySlot);

        mLeftWheelsMaster.enableVoltageCompensation(mConfig.kLeftMaxVoltage);
        mRightWheelsMaster.enableVoltageCompensation(mConfig.kRightMaxVoltage);

//        TODO: See how this affects velocity control on Ramsete
//        mLeftWheelsMaster.setControlFramePeriodMs(1);
//        mRightWheelsMaster.setControlFramePeriodMs(1);
    }

    private void handleBurnToFlash() {
        // Only burn flash in competition to save read/write cycles
        if (kIsInCompetition) {
            mLeftWheelsMaster.burnFlash();
            mLeftWheelsSlave.burnFlash();
            mRightWheelsSlave.burnFlash();
            mRightWheelsMaster.burnFlash();
        }
    }

    @Override
    public void periodic() {
        if (kIsInTuningMode) {
            mLeftWheelsMaster.getPIDController().setP(SmartDashboard.getNumber(kDrivetrainLeftVelocityPKey,
                    mLeftVelocityPID.getP()), kDrivetrainVelocitySlot);

            mRightWheelsMaster.getPIDController().setP(SmartDashboard.getNumber(kDrivetrainRightVelocityPKey,
                    mRightVelocityPID.getP()), kDrivetrainVelocitySlot);
        }

        SmartDashboard.putNumber(kDrivetrainAngularVelocityKey, getAngularVelocityDegreesPerSec());
        SmartDashboard.putNumber(kDrivetrainHeadingKey, getHeading().getDegrees());
        SmartDashboard.putNumber(kDrivetrainLeftPositionKey, getLeftPositionMeters());
        SmartDashboard.putNumber(kDrivetrainRightPositionKey, getRightPositionMeters());
        SmartDashboard.putNumber(kDrivetrainLeftVelocityKey, getLeftVelocityMetersPerSec());
        SmartDashboard.putNumber(kDrivetrainRightVelocityKey, getRightVelocityMetersPerSec());

        updateRobotPose();
        LiveDashboardHelper.putRobotData(getCurrentPose());
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
        mLeftWheelsMaster.getPIDController().setReference(leftVelocity, ControlType.kVelocity, kDrivetrainVelocitySlot,
                leftFeedForward, CANPIDController.ArbFFUnits.kVoltage);
        mRightWheelsMaster.getPIDController().setReference(rightVelocity, ControlType.kVelocity, kDrivetrainVelocitySlot,
                rightFeedForward, CANPIDController.ArbFFUnits.kVoltage);
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
        mAhrs.zeroYaw();
    }

    public void resetPose(Pose2d startingPose) {
        resetEncoders();
        mDriveOdometry.resetPosition(startingPose, getHeading());
    }

    public SimpleMotorFeedforward getFeedforward() {
        return mFeedforward;
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

    public PIDController getLeftVoltagePID() {
        return mLeftVoltagePID;
    }

    public PIDController getRightVoltagePID() {
        return mRightVoltagePID;
    }

    public PIDController getLeftVelocityPID() {
        return mLeftVelocityPID;
    }

    public PIDController getRightVelocityPID() {
        return mRightVelocityPID;
    }

    public ProfiledPIDController getTurnProfiledPID() {
        return mTurnProfiledPID;
    }

    public ProfiledPIDController getDriveStraightPowerProfiledPID() {
        return mDriveStraightPowerProfiledPID;
    }

    public ProfiledPIDController getDriveStraightHeadingProfiledPID() {
        return mDriveStraightHeadingProfiledPID;
    }

    public LinearSystemLoop<N2, N2, N2> getDriveControlLoop() {
        return mDriveControlLoop;
    }

    public Pose2d getCurrentPose() {
        return mCurrentPose;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(),
                getRightVelocityMetersPerSec()
        );
    }

    public double getLeftPositionMeters() {
        return mLeftWheelsMaster.getEncoder().getPosition();
    }

    public double getRightPositionMeters() {
        return mRightWheelsMaster.getEncoder().getPosition();
    }

    public double getLeftVelocityMetersPerSec() {
        return mLeftWheelsMaster.getEncoder().getVelocity();
    }

    public double getRightVelocityMetersPerSec() {
        return mRightWheelsMaster.getEncoder().getVelocity();
    }

    public double getLeftVoltage() {
        return mLeftWheelsMaster.getBusVoltage() * mLeftWheelsMaster.getAppliedOutput();
    }

    public double getRightVoltage() {
        return mRightWheelsMaster.getBusVoltage() * mRightWheelsMaster.getAppliedOutput();
    }

    private void updateRobotPose() {
        mCurrentPose = mDriveOdometry.update(getHeading(), getLeftPositionMeters(), getRightPositionMeters());
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-mAhrs.getAngle());
    }

    public double getAngularVelocityDegreesPerSec() {
        return -mAhrs.getRate();
    }

    public DrivetrainConfig getConfig() {
        return mConfig;
    }

}
