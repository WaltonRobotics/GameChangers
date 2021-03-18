package frc.robot.config;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

public class TurretConfig {

    // Limits
    public LimitSwitchNormal kForwardLimitSwitchNormal;
    public LimitSwitchNormal kReverseLimitSwitchNormal;
    public double kForwardSoftLimitRawUnits;
    public double kReverseSoftLimitRawUnits;

    // Positional PID Control
    public double kP;
    public double kI;
    public double kD;
    public double kAllowableError;
    public double kIZone;
    public double kMaxIntegralAccumulator;
    public double kPeakOutput;

    // Motion Magic
    public double kF;
    public double kAcceleration;
    public double kCruiseVelocity;
    public double kSCurveStrength;

}
