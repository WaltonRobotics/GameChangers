package frc.robot.config;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class DrivetrainConfig {

    public SimpleMotorFeedforward feedforward;

    public double kLeftMaxVoltage;
    public double kRightMaxVoltage;

    public PIDController leftVoltagePID;
    public PIDController rightVoltagePID;

    public PIDController leftVelocityPID;
    public PIDController rightVelocityPID;

    public ProfiledPIDController turnProfiledPID;
    public ProfiledPIDController driveStraightProfiledPowerPID;
    public ProfiledPIDController driveStraightProfiledHeadingPID;

    public double kPositionFactor;
    public double kVelocityFactor;

    public double kTrackWidth;

    public double kMaxVelocity;
    public double kMaxAcceleration;

}
