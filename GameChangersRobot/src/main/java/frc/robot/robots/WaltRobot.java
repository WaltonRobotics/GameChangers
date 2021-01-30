package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public interface WaltRobot {

    /* Drivetrain methods */
    SimpleMotorFeedforward getDrivetrainFeedforward();
    PIDController getDrivetrainLeftVoltagePID();
    PIDController getDrivetrainRightVoltagePID();
    PIDController getDrivetrainLeftVelocityPID();
    PIDController getDrivetrainRightVelocityPID();
    double getDrivetrainPositionFactor();
    double getDrivetrainVelocityFactor();

    double getTrackWidth();

}
