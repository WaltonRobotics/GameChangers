package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.movingAverage.SimpleMovingAverage;

import java.util.logging.Level;

import static frc.robot.Constants.Tuning.kDrivetrainAccelerationWindow;
import static frc.robot.Robot.sDrivetrain;

public class FindMaxVelAccel extends CommandBase {

    private final Timer mTimer;
    private final double mTotalTime;
    private final SimpleMovingAverage mAccelerationMovingAverage
            = new SimpleMovingAverage(kDrivetrainAccelerationWindow);

    private double mPreviousVelocity;
    private double mPrevTime;

    private double mCurrentMaxVelocity;
    private double mCurrentMaxAcceleration;

    public FindMaxVelAccel(double time) {
        addRequirements(sDrivetrain);

        mTimer = new Timer();
        mTotalTime = time;

        DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                "The FindMaxVelAccel command will run the robot at its maximum velocity");
    }

    @Override
    public void initialize() {
        mTimer.start();
    }

    @Override
    public void execute() {
        sDrivetrain.setDutyCycles(1.0, 1.0);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return mTimer.get() > mTotalTime;
    }
}
