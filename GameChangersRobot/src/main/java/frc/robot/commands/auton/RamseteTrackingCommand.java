package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.auton.LiveDashboardHelper;
import frc.robot.auton.LiveDashboardTable;
import frc.robot.auton.RamseteDebuggingTable;

import java.util.function.Supplier;

import static frc.robot.Robot.sDrivetrain;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteTrackingCommand extends CommandBase {
    private final Timer mTimer = new Timer();
    private final boolean mUseSparkPID;
    private final Supplier<Trajectory> mTrajectorySupplier;
    private final Supplier<Pose2d> mPose;
    private final RamseteController mFollower;
    private final SimpleMotorFeedforward mFeedforward;
    private final DifferentialDriveKinematics mKinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> mSpeeds;
    private final PIDController mLeftController;
    private final PIDController mRightController;
    private DifferentialDriveWheelSpeeds mPrevSpeeds;
    private Trajectory mTrajectory;
    private double mPrevTime;

    /**
     * Constructs a new RamseteTrackingCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * in meters per second from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectorySupplier A supplier that points to the trajectory to follow.
     * @param useSparkPID        Whether to use onboard SparkMax velocity PID or direct voltage control
     * @param disableRamsete     Whether to disable the Ramsete correction (for feedforward analysis)
     */
    public RamseteTrackingCommand(Supplier<Trajectory> trajectorySupplier, boolean useSparkPID, boolean disableRamsete) {
        addRequirements(sDrivetrain);

        mTrajectorySupplier = trajectorySupplier;
        mPose = sDrivetrain::getCurrentPose;

        if (disableRamsete) {
            mFollower = new RamseteController() {
                @Override
                public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                                               double angularVelocityRefRadiansPerSecond) {
                    return new ChassisSpeeds(linearVelocityRefMeters,
                            0.0,
                            angularVelocityRefRadiansPerSecond);
                }
            };
        } else {
            mFollower = sDrivetrain.getRamseteController();
        }

        mKinematics = sDrivetrain.getDriveKinematics();

        mFeedforward = sDrivetrain.getLinearFeedforward();
        mSpeeds = sDrivetrain::getSpeeds;
        mLeftController = sDrivetrain.getLeftVelocityPID();
        mRightController = sDrivetrain.getRightVelocityPID();

        mUseSparkPID = useSparkPID;
    }

    /**
     * Constructs a new RamseteTrackingCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * in meters per second from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory     The trajectory to follow.
     * @param useSparkPID    Whether to use onboard SparkMax velocity PID or direct voltage control
     * @param disableRamsete Whether to disable the Ramsete correction (for feedforward analysis)
     */
    public RamseteTrackingCommand(Trajectory trajectory, boolean useSparkPID, boolean disableRamsete) {
        this(() -> trajectory, useSparkPID, disableRamsete);
    }

    @Override
    public void initialize() {
        mTrajectory = mTrajectorySupplier.get();

        mPrevTime = -1;
        var initialState = mTrajectory.sample(0);
        mPrevSpeeds = mKinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        mTimer.reset();
        mTimer.start();
        if (!mUseSparkPID) {
            mLeftController.reset();
            mRightController.reset();
        }

        LiveDashboardTable.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(sDrivetrain.getCurrentPose());
        LiveDashboardHelper.putTrajectoryData(mTrajectory.getInitialPose());

        sDrivetrain.getDriveControlLoop().reset(
                VecBuilder.fill(mSpeeds.get().leftMetersPerSecond, mSpeeds.get().rightMetersPerSecond)
        );

        sDrivetrain.getCrossCoupledFeedforward().reset(
                VecBuilder.fill(mSpeeds.get().leftMetersPerSecond, mSpeeds.get().rightMetersPerSecond)
        );
    }

    @Override
    public void execute() {
        double curTime = mTimer.get();
        double dt = curTime - mPrevTime;

        if (mPrevTime < 0) {
            sDrivetrain.setVelocities(0, 0, 0, 0);
            mPrevTime = curTime;
            return;
        }

        var targetWheelSpeeds = mKinematics.toWheelSpeeds(
                mFollower.calculate(mPose.get(), mTrajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (!mUseSparkPID) {
//            double leftFeedforward =
//                    mFeedforward.calculate(leftSpeedSetpoint,
//                            (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt);
//
//            double rightFeedforward =
//                    mFeedforward.calculate(rightSpeedSetpoint,
//                            (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt);
//
//            leftOutput = leftFeedforward
//                    + mLeftController.calculate(mSpeeds.get().leftMetersPerSecond,
//                    leftSpeedSetpoint);
//
//            rightOutput = rightFeedforward
//                    + mRightController.calculate(mSpeeds.get().rightMetersPerSecond,
//                    rightSpeedSetpoint);
//
//            sDrivetrain.setVoltages(leftOutput, rightOutput);

            sDrivetrain.getDriveControlLoop().setNextR(VecBuilder.fill(leftSpeedSetpoint, rightSpeedSetpoint));
            sDrivetrain.getDriveControlLoop().correct(
                    VecBuilder.fill(mSpeeds.get().leftMetersPerSecond, mSpeeds.get().rightMetersPerSecond)
            );
            sDrivetrain.getDriveControlLoop().predict(dt);

            sDrivetrain.getCrossCoupledFeedforward().calculate(VecBuilder.fill(leftSpeedSetpoint, rightSpeedSetpoint));

            double leftFeedforward = sDrivetrain.getCrossCoupledFeedforward().getUff(0);
            double rightFeedforward = sDrivetrain.getCrossCoupledFeedforward().getUff(1);

            leftOutput = leftFeedforward + sDrivetrain.getDriveControlLoop().getU(0);
            rightOutput = rightFeedforward + sDrivetrain.getDriveControlLoop().getU(1);

            sDrivetrain.setVoltages(leftOutput, rightOutput);
        } else {
//            double leftFeedforward =
//                    mFeedforward.calculate(leftSpeedSetpoint,
//                            (leftSpeedSetpoint - mPrevSpeeds.leftMetersPerSecond) / dt);
//
//            double rightFeedforward =
//                    mFeedforward.calculate(rightSpeedSetpoint,
//                            (rightSpeedSetpoint - mPrevSpeeds.rightMetersPerSecond) / dt);

            sDrivetrain.getCrossCoupledFeedforward().calculate(VecBuilder.fill(leftSpeedSetpoint, rightSpeedSetpoint));

            double leftFeedforward = sDrivetrain.getCrossCoupledFeedforward().getUff(0);
            double rightFeedforward = sDrivetrain.getCrossCoupledFeedforward().getUff(1);

//            System.out.println(leftFeedforward);
//            System.out.println(rightFeedforward);

            RamseteDebuggingTable.getInstance().setLeftMeasurement(mSpeeds.get().leftMetersPerSecond);
            RamseteDebuggingTable.getInstance().setLeftReference(leftSpeedSetpoint);

            RamseteDebuggingTable.getInstance().setRightMeasurement(mSpeeds.get().rightMetersPerSecond);
            RamseteDebuggingTable.getInstance().setRightReference(rightSpeedSetpoint);

            sDrivetrain.setVelocities(leftSpeedSetpoint, leftFeedforward, rightSpeedSetpoint, rightFeedforward);
        }

        mPrevTime = curTime;
        mPrevSpeeds = targetWheelSpeeds;

        LiveDashboardHelper.putRobotData(sDrivetrain.getCurrentPose());
        LiveDashboardHelper.putTrajectoryData(mTrajectory.sample(curTime).poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
        LiveDashboardTable.getInstance().setFollowingPath(false);

        if (interrupted) {
            sDrivetrain.setVelocities(0.0, 0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return mTimer.hasPeriodPassed(mTrajectory.getTotalTimeSeconds());
    }
}