package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

public class PracticeGameChangers implements WaltRobot {

    private final SimpleMotorFeedforward mDrivetrainFeedforward = new SimpleMotorFeedforward(0.144, 2.09, 0.558);
    private final PIDController mDrivetrainLeftVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainRightVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainLeftVelocityPID = new PIDController(0.197, 0, 0);
    private final PIDController mDrivetrainRightVelocityPID = new PIDController(0.197, 0, 0);

    private final double[][] mDistanceToVelocityTable = {
            { 10.03, 12650 },
            { 12.72, 12300 },
            { 15.3, 12350 },
            { 17.17, 12650 },
            { 18.9, 12650 },
            { 19.97, 12750 },
            { 8.38, 12700 },
            { 22.9, 13400 },
            { 23.2, 13800 }
    };

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterMap;
    private final PolynomialRegression mShooterPolynomial;

    public PracticeGameChangers() {
        mShooterMap = new InterpolatingTreeMap<>();
        mShooterPolynomial = new PolynomialRegression(new double[][]{}, 2);

        populateShooterInterpolationMethods();
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
    public double getDrivetrainPositionFactor() {
        return 0.05984734;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return getDrivetrainPositionFactor() / 60.;
    }

    @Override
    public double getTrackWidth() {
        return 0.6644190927877744;
    }

    @Override
    public double getLimelightMountingHeight() {
        return 24;
    }

    @Override
    public double getLimelightMountingAngle() {
        return 35;
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
}
