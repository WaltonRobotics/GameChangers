package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import frc.robot.config.SwerveDriveConfig;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;

import static frc.robot.config.SwerveDriveConfig.kTalonConfigTimeout;

public class SwerveDrivetrain extends SubsystemBase {
    private static SwerveDrive swerveDrive;

    public SwerveDrivetrain() {
        var moduleBuilder =
                new TalonSwerveModule.Builder()
                        .driveGearRatio(SwerveDriveConfig.kDriveGearRatio)
                        .wheelDiameterInches(SwerveDriveConfig.kWheelDiameterInches)
                        .driveMaximumMetersPerSecond(SwerveDriveConfig.kMaxSpeedMetersPerSecond);

        TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
        Translation2d[] wheelLocations = SwerveDriveConfig.getWheelLocationMeters();


        for (int i = 0; i < 4; i++) {
            var azimuthTalon = new TalonSRX(i);
            azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
            azimuthTalon.configAllSettings(SwerveDriveConfig.getAzimuthTalonConfig(), kTalonConfigTimeout);
            azimuthTalon.enableCurrentLimit(true);
            azimuthTalon.enableVoltageCompensation(true);
            azimuthTalon.setNeutralMode(NeutralMode.Coast);

            var driveTalon = new TalonFX(i + 10);
            driveTalon.configFactoryDefault(kTalonConfigTimeout);
            driveTalon.configAllSettings(SwerveDriveConfig.getDriveTalonConfig(), kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            swerveModules[i] =
                    moduleBuilder
                            .azimuthTalon(azimuthTalon)
                            .driveTalon(driveTalon)
                            .wheelLocationMeters(wheelLocations[i])
                            .build();

            swerveModules[i].loadAndSetAzimuthZeroReference();
        }

        swerveDrive = new SwerveDrive(swerveModules);
        swerveDrive.resetGyro();
        swerveDrive.setGyroOffset(Rotation2d.fromDegrees(180));
    }

    /**
     * Returns the swerve drive kinematics object for use during trajectory configuration.
     *
     * @return the configured kinemetics object
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDrive.getKinematics();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public SwerveModule[] getSwerveModules() {
        return swerveDrive.getSwerveModules();
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose the current pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return swerveDrive.getPoseMeters();
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
        swerveDrive.periodic();
    }

    /**
     * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
     */
    public static void drive(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /**
     * Move the robot with given x, y, and rotational velocities with closed-loop velocity control.
     */
    public void move(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            boolean isFieldOriented) {
        swerveDrive.move(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    }

    public void resetGyro() {
        swerveDrive.resetGyro();
    }

    public void setGyroOffset(Rotation2d offsetRads) {
        swerveDrive.setGyroOffset(offsetRads);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getHeading();
    }

    public void xLockSwerveDrive() {
        ((TalonSwerveModule) swerveDrive.getSwerveModules()[0])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        ((TalonSwerveModule) swerveDrive.getSwerveModules()[1])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((TalonSwerveModule) swerveDrive.getSwerveModules()[2])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((TalonSwerveModule) swerveDrive.getSwerveModules()[3])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
    }
}