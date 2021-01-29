package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class PracticeGameChangers implements WaltRobot {

    SimpleMotorFeedforward drivetrainFeedforward = new SimpleMotorFeedforward(0.147, 2.1, 0.561);
    PIDController drivetrainVoltagePID = new PIDController(1, 0, 0);
    PIDController drivetrainVelocityPID= new PIDController(2.37, 0, 0);

    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return drivetrainFeedforward;
    }

    @Override
    public PIDController getDrivetrainVoltagePID() {
        return drivetrainVoltagePID;
    }

    @Override
    public PIDController getDrivetrainVelocityPID() {
        return drivetrainVelocityPID;
    }

    @Override
    public double getDrivetrainPositionFactor() {
        return 1/16.773784;
    }

    @Override
    public double getDrivetrainVelocityFactor() {
        return getDrivetrainPositionFactor() / 60.;
    }

    @Override
    public double getTrackWidth() {
        return 0.6692607968397127;
    }
}
