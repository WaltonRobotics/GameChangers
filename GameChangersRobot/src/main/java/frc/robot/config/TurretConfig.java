package frc.robot.config;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class TurretConfig {

    public double kTicksPerDegree;
    public Rotation2d kLimitSwitchPosition;

    // Limits
    public double kForwardSoftLimitRawUnits;
    public double kReverseSoftLimitRawUnits;

    // Positional PID Control
    public double kPositionalP;
    public double kPositionalI;
    public double kPositionalD;
    public double kPositionalIZone;
    public double kPositionalMaxIntegralAccumulator;

    // Motion Magic
    public double kMotionMagicF;
    public double kMotionMagicP;
    public double kMotionMagicI;
    public double kMotionMagicD;
    public double kMotionMagicIZone;
    public double kMotionMagicMaxIntegralAccumulator;
    public double kAcceleration;
    public double kCruiseVelocity;
    public int kSCurveStrength;

    public ProfiledPIDController closedLoopAutoAlignProfiledPID;

}
