package frc.robot.robots;

import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.regression.PolynomialRegression;

/* Generic superclass for all Walton robots and their characteristics. */
public abstract class WaltRobot {

    protected final DrivetrainConfig mDrivetrainConfig;
    protected final ShooterConfig mShooterConfig;
    protected final IntakeConfig mIntakeConfig;
    protected final ConveyorConfig mConveyorConfig;
    protected final TurretConfig mTurretConfig;

    protected final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterMap;
    protected final PolynomialRegression mShooterPolynomial;

    public WaltRobot(double[][] distanceToVelocityTable, int shooterPolynomialDegree) {
        mDrivetrainConfig = new DrivetrainConfig();
        mShooterConfig = new ShooterConfig();
        mIntakeConfig = new IntakeConfig();
        mConveyorConfig = new ConveyorConfig();
        mTurretConfig = new TurretConfig();

        mShooterMap = new InterpolatingTreeMap<>();
        mShooterPolynomial = new PolynomialRegression(new double[][]{}, shooterPolynomialDegree);

        mShooterConfig.kShooterMap = mShooterMap;
        mShooterConfig.kShooterPolynomial = mShooterPolynomial;

        mShooterPolynomial.fit(distanceToVelocityTable);

        mShooterMap.clear();
        for (double[] pair : distanceToVelocityTable) {
            mShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public DrivetrainConfig getDrivetrainConfig() {
        return mDrivetrainConfig;
    }

    public ShooterConfig getShooterConfig() {
        return mShooterConfig;
    }

    public IntakeConfig getIntakeConfig() {
        return mIntakeConfig;
    }

    public ConveyorConfig getConveyorConfig() {
        return mConveyorConfig;
    }

    public TurretConfig getTurretConfig() {
        return mTurretConfig;
    }

}
