package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class CompDeepSpace implements WaltRobot {

    private final SimpleMotorFeedforward mDrivetrainFeedforward = new SimpleMotorFeedforward(0.178, 3.19, 0.462);
    private final PIDController mDrivetrainLeftVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainRightVoltagePID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainLeftVelocityPID = new PIDController(1, 0, 0);
    private final PIDController mDrivetrainRightVelocityPID = new PIDController(1, 0, 0);

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
        return 1/64.125;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return getDrivetrainPositionFactor() / 60.;
    }

    @Override
    public double getTrackWidth() {
        return 0.78;
    }

    public double getLimelightMountingHeight() { return 0; }

    public double getLimelightMountingAngle() { return 0; }

}
