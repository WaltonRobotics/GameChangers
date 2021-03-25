package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.movingAverage.SimpleMovingAverage;

import java.util.logging.Level;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Tuning.kDrivetrainAccelerationWindow;
import static frc.robot.Robot.sDrivetrain;

public class FindAngularMaxVelAccel extends CommandBase {

    private final Timer mTimer;
    private final double mTotalTime;
    private final SimpleMovingAverage mAccelerationMovingAverage
            = new SimpleMovingAverage(kDrivetrainAccelerationWindow);

    private double mPreviousTime;
    private double mPreviousVelocity;

    private double mCurrentMaxVelocity;
    private double mCurrentMaxAcceleration;

    public FindAngularMaxVelAccel(double time) {
        addRequirements(sDrivetrain);

        mTimer = new Timer();
        mTotalTime = time;

        DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                "The FindMaxVelAccel command will run the robot at its maximum velocity");

        mPreviousTime = getFPGATimestamp();
        mPreviousVelocity = getInstantaneousAngularVelocity();
        mCurrentMaxVelocity = 0;
        mCurrentMaxAcceleration = 0;
    }

    @Override
    public void initialize() {
        mTimer.start();
    }

    @Override
    public void execute() {
        sDrivetrain.setDutyCycles(-1.0, 1.0);

        double mCurrentTime = getFPGATimestamp();
        double mCurrentVelocity = getInstantaneousAngularVelocity();

        double instantaneousAcceleration = (mCurrentVelocity - mPreviousVelocity) / (mCurrentTime - mPreviousTime);

        mAccelerationMovingAverage.addData(instantaneousAcceleration);

        if (mCurrentVelocity > mCurrentMaxVelocity) {
            mCurrentMaxVelocity = mCurrentVelocity;
        }

        if (mAccelerationMovingAverage.getMean() > mCurrentMaxAcceleration) {
            mCurrentMaxAcceleration = mAccelerationMovingAverage.getMean();
        }

        mPreviousTime = mCurrentTime;
        mPreviousVelocity = mCurrentVelocity;
    }

    @Override
    public void end(boolean interrupted) {
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Max velocity: " + mCurrentMaxVelocity + " m/s and max acceleration: "
                        + mCurrentMaxAcceleration + " m/s^2");
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() > mTotalTime;
    }

    private double getInstantaneousAngularVelocity() {
        return sDrivetrain.getAngularVelocityDegreesPerSec();
    }

}
