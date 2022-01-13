package frc.robot.config;

import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.regression.PolynomialRegression;

public class ShooterConfig {

    // Velocity PIDF Control
    public double kSpinningUpF;
    public double kSpinningUpP;
    public double kSpinningUpI;
    public double kSpinningUpD;
    public double kSpinningUpIZone;
    public double kSpinningUpMaxIntegralAccumulator;

    public double kShootingF;
    public double kShootingP;
    public double kShootingI;
    public double kShootingD;
    public double kShootingIZone;
    public double kShootingMaxIntegralAccumulator;

    public double kMaxVoltage;

    // Limelight
    public double kLimelightMountingHeightInches;
    public double kLimelightMountingAngleDegrees;

    public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kShooterMap;
    public PolynomialRegression kShooterPolynomial;

}
