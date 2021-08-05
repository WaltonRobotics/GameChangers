package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.auton.LiveDashboardHelper;
import frc.robot.config.DrivetrainConfig;
import frc.robot.robots.RobotIdentifier;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;

import java.util.Arrays;
import java.util.logging.Level;

import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.ContextFlags.kIsInCompetition;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.PIDSlots.kDrivetrainVelocitySlot;
import static frc.robot.Constants.PIDSlots.kDrivetrainVoltageSlot;
import static frc.robot.Constants.PneumaticsIDs.kDrivetrainGearShiftSolenoidID;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.Tuning.kDrivetrainTuningSettingsUpdateRateSeconds;
import static frc.robot.Robot.sCurrentRobot;

public class SwerveDrivetrain extends SubsystemBase {

    private final DrivetrainConfig mConfig = sCurrentRobot.getCurrentRobot().getDrivetrainConfig();

    // create telemetry first
    private final TelemetryService telemetryService = RobotContainer.TELEMETRY;
    private final SwerveDrive swerve = getSwerve();

    private final Logger logger = LoggerFactory.getLogger(this.getClass());

    public DriveSubsystem() {
        swerve.setFieldOriented(true);
        zeroAzimuths();
    }

    public void zeroAzimuths() {
        swerve.zeroAzimuthEncoders();
    }

    public void drive(double forward, double strafe, double yaw) {
        swerve.drive(forward, strafe, yaw);
    }

    public void zeroGyro() {
        AHRS gyro = swerve.getGyro();
        gyro.setAngleAdjustment(0);
        double adj = gyro.getAngle() % 360;
        gyro.setAngleAdjustment(-adj);
        logger.info("resetting gyro: ({})", adj);
    }

    // Swerve configuration

    private SwerveDrive getSwerve() {
        SwerveDriveConfig config = new SwerveDriveConfig();
        config.wheels = getWheels();
        config.gyro = new AHRS(SPI.Port.kMXP);
        config.length = ROBOT_LENGTH;
        config.width = ROBOT_WIDTH;
        config.gyroLoggingEnabled = true;
        config.summarizeTalonErrors = false;

        return new SwerveDrive(config);
    }

    private Wheel[] getWheels() {
        TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
        // NOTE: ensure encoders are in-phase with motor direction. Encoders should increase
        // when azimuth motor runs in forward direction.
        azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        azimuthConfig.continuousCurrentLimit = 10;
        azimuthConfig.peakCurrentDuration = 0;
        azimuthConfig.peakCurrentLimit = 0;
        azimuthConfig.slot0.kP = 20;
        azimuthConfig.slot0.kI = 0.0;
        azimuthConfig.slot0.kD = 300.0;
        azimuthConfig.slot0.kF = 0.0;
        azimuthConfig.slot0.integralZone = 0;
        azimuthConfig.slot0.allowableClosedloopError = 0;
        azimuthConfig.motionAcceleration = 10_000;
        azimuthConfig.motionCruiseVelocity = 800;
        azimuthConfig.peakOutputForward = 0.75;
        azimuthConfig.peakOutputReverse = -0.75;

        TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        driveConfig.continuousCurrentLimit = 40;
        driveConfig.peakCurrentDuration = 0;
        driveConfig.peakCurrentLimit = 0;

        Wheel[] wheels = new Wheel[4];

        for (int i = 0; i < 4; i++) {
            TalonSRX azimuthTalon = new TalonSRX(i);
            azimuthTalon.configAllSettings(azimuthConfig);

            telemetryService.register(azimuthTalon);

            TalonSRX driveTalon = new TalonSRX(i + 10);
            driveTalon.configAllSettings(driveConfig);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            telemetryService.register(driveTalon);

            Wheel wheel = new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
            wheels[i] = wheel;
        }

        return wheels;
    }
}