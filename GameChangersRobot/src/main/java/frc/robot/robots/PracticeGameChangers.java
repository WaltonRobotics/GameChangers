package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

public class PracticeGameChangers implements WaltRobot {

    private final double[][] mDistanceToVelocityTable = {
            { 8.61, 13000 },
            { 10.94, 11500 },
            { 12.83, 11400 },
            { 15.735, 11250 },
            { 17.254, 11350 },
            { 19.22, 11750 },
            { 22.38, 12425 },
    };

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterMap;
    private final PolynomialRegression mShooterPolynomial;

    private final DrivetrainConfig mDrivetrainConfig;
    private final ShooterConfig mShooterConfig;
    private final IntakeConfig mIntakeConfig;
    private final ConveyorConfig mConveyorConfig;
    private final TurretConfig mTurretConfig;

    public PracticeGameChangers() {
        mShooterMap = new InterpolatingTreeMap<>();
        mShooterPolynomial = new PolynomialRegression(new double[][]{}, 2);

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

        mShooterConfig.kOtherZonesShooterMap = mShooterMap;
        mShooterConfig.kOtherZonesShooterPolynomial = mShooterPolynomial;

        mIntakeConfig = new IntakeConfig();
        mIntakeConfig.kIsIntakeControllerInverted = false;
        mIntakeConfig.kIntakeDutyCycle = 1.0;
        mIntakeConfig.kOuttakeDutyCycle = -1.0;
        mIntakeConfig.kSettleTime = 0.5;

        mConveyorConfig = new ConveyorConfig();
        mConveyorConfig.kIsFrontConveyorControllerInverted = true;
        mConveyorConfig.kIsBackConveyorControllerInverted = false;
        mConveyorConfig.kIRSensorFlickeringTimeSeconds = 0.75;
        mConveyorConfig.kNudgeTimeSeconds = 0.29;
        mConveyorConfig.kFrontConveyorNudgeVoltage = 8.0;
        mConveyorConfig.kBackConveyorNudgeVoltage = 8.0;
        mConveyorConfig.kFrontConveyorFeedVoltage = 11.75;
        mConveyorConfig.kBackConveyorFeedVoltage = 11.75;
        mConveyorConfig.kFrontConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kBackConveyorIntakeDutyCycle = 1.0;
        mConveyorConfig.kFrontConveyorOuttakeDutyCycle = -1.0;
        mConveyorConfig.kBackConveyorOuttakeDutyCycle = -1.0;

        mTurretConfig = new TurretConfig();
    }

    @Override
    public void populateShooterInterpolationMethods() {
        mShooterPolynomial.fit(mDistanceToVelocityTable);

        mShooterMap.clear();
        for (double[] pair : mDistanceToVelocityTable) {
            mShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
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
