package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class CompPowerUp implements WaltRobot {
    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return null;
    }

    @Override
    public PIDController getDrivetrainLeftVoltagePID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainRightVoltagePID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainLeftVelocityPID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainRightVelocityPID() {
        return null;
    }

    @Override
    public double getDrivetrainPositionFactor() {
        return 0;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return 0;
    }

    @Override
    public double getTrackWidth() {
        return 0;
    }

    public double getLimelightMountingHeight() { return 0; }

    public double getLimelightMountingAngle() { return 0; }

}