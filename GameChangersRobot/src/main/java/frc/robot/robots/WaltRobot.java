package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public interface WaltRobot {

    /* Drivetrain methods */
    SimpleMotorFeedforward getDrivetrainFeedforward();
    PIDController getDrivetrainVoltagePID();
    PIDController getDrivetrainVelocityPID();
    double getDrivetrainPositionFactor();
    double getDrivetrainVelocityFactor();

}
