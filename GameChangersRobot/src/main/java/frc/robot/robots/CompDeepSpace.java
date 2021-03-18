package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.config.*;
import frc.robot.utils.interpolation.InterpolatingDouble;
import frc.robot.utils.interpolation.InterpolatingTreeMap;
import frc.robot.utils.interpolation.PolynomialRegression;

public class CompDeepSpace implements WaltRobot {

    private final ProfiledPIDController mDrivetrainTurnPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    private final ProfiledPIDController mDrivetrainDriveStraightPowerPID = new ProfiledPIDController(0.8, 0, 0,
            new TrapezoidProfile.Constraints(1.5,1.5));
    private final ProfiledPIDController mDrivetrainDriveStraightHeadingPID = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(60, 30));

    public CompDeepSpace() {
        mDrivetrainTurnPID.setTolerance(6.0);
        mDrivetrainDriveStraightPowerPID.setTolerance(0.09);
        mDrivetrainDriveStraightHeadingPID.setTolerance(3);
    }

    @Override
    public void populateShooterInterpolationMethods() {

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
