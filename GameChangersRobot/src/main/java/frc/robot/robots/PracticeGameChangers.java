package frc.robot.robots;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Units;

public class PracticeGameChangers implements WaltRobot {

    private SimpleMotorFeedforward mDrivetrainFeedforward = new SimpleMotorFeedforward(0.144, 2.09, 0.558);
    private PIDController mDrivetrainLeftVoltagePID = new PIDController(1, 0, 0);
    private PIDController mDrivetrainRightVoltagePID = new PIDController(1, 0, 0);
    private PIDController mDrivetrainLeftVelocityPID = new PIDController(0.05, 0, 0);
    private PIDController mDrivetrainRightVelocityPID = new PIDController(0.05, 0, 0);

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
        return 0.8;
    }
}
