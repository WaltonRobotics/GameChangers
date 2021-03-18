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

    private final ProfiledPIDController mDrivetrainTurnPID = new ProfiledPIDController(0.015, 0, 0,
            new TrapezoidProfile.Constraints(400, 400));

    private final ProfiledPIDController mDrivetrainDriveStraightPowerPID = new ProfiledPIDController(0.8, 0, 0,
            new TrapezoidProfile.Constraints(1.5,1.5));
    private final ProfiledPIDController mDrivetrainDriveStraightHeadingPID = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(60, 30));

    private final SimpleMotorFeedforward mDrivetrainFeedforward = new SimpleMotorFeedforward(0.144, 2.09, 0.558);
    private final PIDController mDrivetrainLeftVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainRightVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainLeftVelocityPID = new PIDController(0.197, 0, 0);
    private final PIDController mDrivetrainRightVelocityPID = new PIDController(0.197, 0, 0);

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

    public PracticeGameChangers() {
        mShooterMap = new InterpolatingTreeMap<>();
        mShooterPolynomial = new PolynomialRegression(new double[][]{}, 2);

        populateShooterInterpolationMethods();

        mDrivetrainTurnPID.enableContinuousInput(-180.0, 180.0);
        mDrivetrainTurnPID.setTolerance(1, 1);

        mDrivetrainDriveStraightPowerPID.setTolerance(0.09);
        mDrivetrainDriveStraightHeadingPID.setTolerance(3);
    }

    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return mDrivetrainFeedforward;
    }

    @Override
    public PIDController getDrivetrainLeftVoltagePID() {
        return mDrivetrainLeftVoltagePID;
    }

    @Override
    public PIDController getDrivetrainRightVoltagePID() {
        return mDrivetrainRightVoltagePID;
    }

    @Override
    public PIDController getDrivetrainLeftVelocityPID() {
        return mDrivetrainLeftVelocityPID;
    }

    @Override
    public PIDController getDrivetrainRightVelocityPID() {
        return mDrivetrainRightVelocityPID;
    }

    @Override
    public ProfiledPIDController getDrivetrainTurnPID() {
        return mDrivetrainTurnPID;
    }

    @Override
    public ProfiledPIDController getDrivetrainDriveStraightPowerPID() {
        return mDrivetrainDriveStraightPowerPID;
    }

    @Override
    public ProfiledPIDController getDrivetrainDriveStraightHeadingPID() {
        return mDrivetrainDriveStraightHeadingPID;
    }

    @Override
    public double getDrivetrainPositionFactor() {
        return 0.05984734;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return getDrivetrainPositionFactor() / 60.;
    }

    @Override
    public double getDrivetrainTrackWidth() {
        return 0.6644190927877744;
    }

    @Override
    public double getDrivetrainMaxVelocity() {
        return 8;
    }

    @Override
    public double getDrivetrainMaxAcceleration() {
        return 3;
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
        return null;
    }

    @Override
    public ShooterConfig getShooterConfig() {
        return null;
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        return null;
    }

    @Override
    public ConveyorConfig getConveyorConfig() {
        return null;
    }

    @Override
    public TurretConfig getTurretConfig() {
        return null;
    }
}
