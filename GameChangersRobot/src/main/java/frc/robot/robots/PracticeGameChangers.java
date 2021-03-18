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

    private final SimpleMotorFeedforward mDrivetrainFeedforward = new SimpleMotorFeedforward(0.144, 2.09, 0.558);
    private final PIDController mDrivetrainLeftVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainRightVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainLeftVelocityPID = new PIDController(0.197, 0, 0);
    private final PIDController mDrivetrainRightVelocityPID = new PIDController(0.197, 0, 0);

    private final ProfiledPIDController mDrivetrainTurnProfiledPID = new ProfiledPIDController(
            0.015, 0, 0,
            new TrapezoidProfile.Constraints(400, 400)
    );

    private final ProfiledPIDController mDrivetrainDriveStraightPowerProfiledPID = new ProfiledPIDController(
            0.8, 0, 0,
            new TrapezoidProfile.Constraints(1.5,1.5)
    );
    private final ProfiledPIDController mDrivetrainDriveStraightHeadingProfiledPID = new ProfiledPIDController(
            0.2, 0, 0,
            new TrapezoidProfile.Constraints(60, 30)
    );

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

        mDrivetrainTurnProfiledPID.enableContinuousInput(-180.0, 180.0);
        mDrivetrainTurnProfiledPID.setTolerance(1, 1);

        mDrivetrainDriveStraightPowerProfiledPID.setTolerance(0.09);
        mDrivetrainDriveStraightHeadingProfiledPID.setTolerance(3);

        mDrivetrainConfig = new DrivetrainConfig();
        mDrivetrainConfig.feedforward = mDrivetrainFeedforward;
        mDrivetrainConfig.leftVoltagePID = mDrivetrainLeftVoltagePID;
        mDrivetrainConfig.rightVoltagePID = mDrivetrainRightVoltagePID;
        mDrivetrainConfig.leftVelocityPID = mDrivetrainLeftVelocityPID;
        mDrivetrainConfig.rightVelocityPID = mDrivetrainRightVelocityPID;
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

        mShooterConfig = new ShooterConfig();
        mIntakeConfig = new IntakeConfig();
        mConveyorConfig = new ConveyorConfig();
        mTurretConfig = new TurretConfig();
    }

    @Override
    public double getLimelightMountingHeight() {
        return 23;
    }

    @Override
    public double getLimelightMountingAngle() {
        return 33.5;
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
    public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap() {
        return mShooterMap;
    }

    @Override
    public PolynomialRegression getShooterPolynomial() {
        return mShooterPolynomial;
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
