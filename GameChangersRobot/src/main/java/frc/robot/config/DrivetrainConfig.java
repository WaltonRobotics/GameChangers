package frc.robot.config;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class DrivetrainConfig {

    public SimpleMotorFeedforward linearFeedforward;
    public SimpleMotorFeedforward angularFeedforward;

    public PIDController leftVoltagePID;
    public PIDController rightVoltagePID;

    public PIDController leftVelocityPID;
    public PIDController rightVelocityPID;

    public ProfiledPIDController turnProfiledPID;
    public ProfiledPIDController driveStraightProfiledPowerPID;
    public ProfiledPIDController driveStraightProfiledHeadingPID;

    public double kLeftMaxVoltage;
    public double kRightMaxVoltage;

    public double kPositionFactor;
    public double kVelocityFactor;

    public double kTrackWidthMeters;

    public double kMaxVelocityMetersPerSecond;
    public double kMaxAccelerationMetersPerSecondSquared;

}
