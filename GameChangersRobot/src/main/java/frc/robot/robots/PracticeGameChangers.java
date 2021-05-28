package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.regression.PolynomialRegression;

public class PracticeGameChangers implements WaltRobot {

    // Shooter LUT when the turret is facing sideways and the adjustable hood is up
    private static final double[][] mZoneOneDistanceToVelocityTable = {
            {6.2, 12000},
            {12.16, 12000},
    };

    // Shooter LUT in all other zones
    private static final double[][] mOtherZonesDistanceToVelocityTable = {
//            {11.06, 11600},
//            {18.73, 11200},
//            {30.41, 11700},

            {8.61, 13000},
            {10.94, 11500},
            {12.83, 11400},
            {15.735, 11250},
            {17.254, 11350},
            {19.22, 11750},
            {22.38, 12425},
    };

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mZoneOneShooterMap;
    private final PolynomialRegression mZoneOneShooterPolynomial;
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mOtherZonesShooterMap;
    private final PolynomialRegression mOtherZonesShooterPolynomial;

    private final DrivetrainConfig mDrivetrainConfig;
    private final ShooterConfig mShooterConfig;
    private final IntakeConfig mIntakeConfig;
    private final ConveyorConfig mConveyorConfig;
    private final TurretConfig mTurretConfig;

    public PracticeGameChangers() {
        mZoneOneShooterMap = new InterpolatingTreeMap<>();
        mZoneOneShooterPolynomial = new PolynomialRegression(new double[][]{}, 2);

        mOtherZonesShooterMap = new InterpolatingTreeMap<>();
        mOtherZonesShooterPolynomial = new PolynomialRegression(new double[][]{}, 2);

        populateShooterInterpolationMethods();

        ProfiledPIDController mDrivetrainTurnProfiledPID = new ProfiledPIDController(
                0.015, 0, 0,
                new TrapezoidProfile.Constraints(400, 400)
        );
        mDrivetrainTurnProfiledPID.enableContinuousInput(-180.0, 180.0);
        mDrivetrainTurnProfiledPID.setTolerance(1, 1);

        ProfiledPIDController mDrivetrainDriveStraightPowerProfiledPID = new ProfiledPIDController(
                0.8, 0, 0,
                new TrapezoidProfile.Constraints(1.5, 1.5)
        );
        mDrivetrainDriveStraightPowerProfiledPID.setTolerance(0.09);

        ProfiledPIDController mDrivetrainDriveStraightHeadingProfiledPID = new ProfiledPIDController(
                0.2, 0, 0,
                new TrapezoidProfile.Constraints(60, 30)
        );
        mDrivetrainDriveStraightHeadingProfiledPID.enableContinuousInput(-180.0, 180.0);
        mDrivetrainDriveStraightHeadingProfiledPID.setTolerance(1.5);

        mDrivetrainConfig = new DrivetrainConfig();
        mDrivetrainConfig.linearFeedforward = new SimpleMotorFeedforward(0.144, 2.09, 0.558);
        mDrivetrainConfig.angularFeedforward = new SimpleMotorFeedforward(1.0, 1.0, 1.0);
        mDrivetrainConfig.leftVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.rightVoltagePID = new PIDController(1, 0, 0);
        mDrivetrainConfig.leftVelocityPID = new PIDController(0.197, 0, 0);
        mDrivetrainConfig.rightVelocityPID = new PIDController(0.197, 0, 0);
        mDrivetrainConfig.turnProfiledPID = mDrivetrainTurnProfiledPID;
        mDrivetrainConfig.driveStraightProfiledPowerPID = mDrivetrainDriveStraightPowerProfiledPID;
        mDrivetrainConfig.driveStraightProfiledHeadingPID = mDrivetrainDriveStraightHeadingProfiledPID;
        mDrivetrainConfig.kPositionFactor = 0.05984734;
        mDrivetrainConfig.kVelocityFactor = mDrivetrainConfig.kPositionFactor / 60.0;
        mDrivetrainConfig.kTrackWidthMeters = 0.6644190927877744;
        mDrivetrainConfig.kMaxVelocityMetersPerSecond = 8.0;
        mDrivetrainConfig.kMaxAccelerationMetersPerSecondSquared = 3.0;
        mDrivetrainConfig.kLeftMaxVoltage = 12.0;
        mDrivetrainConfig.kRightMaxVoltage = 12.0;
        mDrivetrainConfig.kOpenLoopRampRate = 0.0;

        mShooterConfig = new ShooterConfig();
        mShooterConfig.kSpinningUpF = 0.04934694;
        mShooterConfig.kSpinningUpP = 0.2;
        mShooterConfig.kSpinningUpI = 0.002;
        mShooterConfig.kSpinningUpD = 0;
        mShooterConfig.kSpinningUpIZone = 600;
        mShooterConfig.kSpinningUpMaxIntegralAccumulator = 1000;

        mShooterConfig.kShootingF = 0.04934694;
        mShooterConfig.kShootingP = 0.21;
        mShooterConfig.kShootingI = 0.002;
        mShooterConfig.kShootingD = 0;
        mShooterConfig.kShootingIZone = 600;
        mShooterConfig.kShootingMaxIntegralAccumulator = 1000;

        mShooterConfig.kMaxVoltage = 11.0;

        mShooterConfig.kLimelightMountingHeight = 23;
        mShooterConfig.kLimelightMountingAngle = 33.5;

        mShooterConfig.kZoneOneShooterMap = mZoneOneShooterMap;
        mShooterConfig.kZoneOneShooterPolynomial = mZoneOneShooterPolynomial;
        mShooterConfig.kOtherZonesShooterMap = mOtherZonesShooterMap;
        mShooterConfig.kOtherZonesShooterPolynomial = mOtherZonesShooterPolynomial;

        mIntakeConfig = new IntakeConfig();
        mIntakeConfig.kIsIntakeControllerInverted = false;
        mIntakeConfig.kIntakeDutyCycle = 0.8;
        mIntakeConfig.kOuttakeDutyCycle = -0.4;
        mIntakeConfig.kSettleTime = 0.5;

        mConveyorConfig = new ConveyorConfig();
        mConveyorConfig.kIsFrontConveyorControllerInverted = true;
        mConveyorConfig.kIsBackConveyorControllerInverted = false;
        mConveyorConfig.kIRSensorFlickeringTimeSeconds = 0.75;
        mConveyorConfig.kNudgeTimeSeconds = 0.25;
        mConveyorConfig.kFrontConveyorNudgeVoltage = 12.0;
        mConveyorConfig.kBackConveyorNudgeVoltage = 10.0;
        mConveyorConfig.kFrontConveyorFeedVoltage = 10.0;
        mConveyorConfig.kBackConveyorFeedVoltage = 12.0;
        mConveyorConfig.kFrontConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kBackConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kFrontConveyorOuttakeDutyCycle = -1.0;
        mConveyorConfig.kBackConveyorOuttakeDutyCycle = -1.0;

        mTurretConfig = new TurretConfig();
        mTurretConfig.kLimitSwitchPosition = Rotation2d.fromDegrees(90);
        mTurretConfig.kForwardSoftLimitRawUnits = 0;
        mTurretConfig.kReverseSoftLimitRawUnits = -460;
        final double kGearRatio = 230.0 / 30.0;
        final double kTicksPerDriverRotation = 177;
        mTurretConfig.kTicksPerDegree = kTicksPerDriverRotation * kGearRatio / 360.0;
        mTurretConfig.kPositionalP = 4.0;
        mTurretConfig.kPositionalI = 0.002;
        mTurretConfig.kPositionalD = 0.0;
        mTurretConfig.kPositionalIZone = 100;
        mTurretConfig.kPositionalMaxIntegralAccumulator = 0;
    }

    @Override
    public void populateShooterInterpolationMethods() {
        mZoneOneShooterPolynomial.fit(mZoneOneDistanceToVelocityTable);

        mZoneOneShooterMap.clear();
        for (double[] pair : mZoneOneDistanceToVelocityTable) {
            mZoneOneShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        mOtherZonesShooterPolynomial.fit(mOtherZonesDistanceToVelocityTable);

        mOtherZonesShooterMap.clear();
        for (double[] pair : mOtherZonesDistanceToVelocityTable) {
            mOtherZonesShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return mDrivetrainConfig;
    }

    @Override
    public ShooterConfig getShooterConfig() {
        return mShooterConfig;
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        return mIntakeConfig;
    }

    @Override
    public ConveyorConfig getConveyorConfig() {
        return mConveyorConfig;
    }

    @Override
    public TurretConfig getTurretConfig() {
        return mTurretConfig;
    }
}
