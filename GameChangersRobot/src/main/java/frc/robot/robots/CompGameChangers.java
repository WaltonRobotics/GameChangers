package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import static frc.robot.robots.RobotIdentification.COMP_DEEP_SPACE;

public class CompGameChangers implements WaltRobot {
    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return null;
    }

    @Override
    public PIDController getDrivetrainVoltagePID() {
        return null;
    }

    @Override
    public PIDController getDrivetrainVelocityPID() {
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
}
