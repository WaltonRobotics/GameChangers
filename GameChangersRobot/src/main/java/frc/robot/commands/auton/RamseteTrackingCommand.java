package frc.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.utils.LiveDashboardHelper;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;

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
    private final Timer m_timer = new Timer();
    private final boolean m_useSparkPID;
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
    private boolean debugMode;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * in meters per second from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory The trajectory to follow.
     */
    public RamseteTrackingCommand(Trajectory trajectory, boolean useSparkPID, boolean debugMode) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        m_pose = drivetrain::getCurrentPose;

        if(debugMode) {
            m_follower = new RamseteController() {
                @Override
                public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                                               double angularVelocityRefRadiansPerSecond) {
                    return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
                }
            };
        } else {
            m_follower = drivetrain.getRamseteController();
        }

        m_kinematics = drivetrain.getDriveKinematics();

        m_feedforward = currentRobot.getCurrentRobot().getDrivetrainFeedforward();
        m_speeds = drivetrain::getSpeeds;
        m_leftController = currentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID();
        m_rightController = currentRobot.getCurrentRobot().getDrivetrainRightVelocityPID();

        m_useSparkPID = useSparkPID;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        if (!m_useSparkPID) {
            m_leftController.reset();
            m_rightController.reset();
        }

        LiveDashboard.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(drivetrain.getCurrentPose());
        LiveDashboardHelper.putTrajectoryData(m_trajectory.getInitialPose());
    }

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    private NetworkTableEntry leftReference = table.getEntry("left_reference");
    private NetworkTableEntry leftMeasurement = table.getEntry("left_measurement");
    private NetworkTableEntry rightReference = table.getEntry("right_reference");
    private NetworkTableEntry rightMeasurement = table.getEntry("right_measurement");

    @Override
    public void execute() {

        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (!m_useSparkPID) {
            /*
            double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    m_feedforward.calculate(rightSpeedSetpoint,
                            (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
                    leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
                    rightSpeedSetpoint);

            drivetrain.setVoltages(leftOutput, rightOutput);
            */

            drivetrain.getDriveControlLoop().setNextR(VecBuilder.fill(leftSpeedSetpoint, rightSpeedSetpoint));
            drivetrain.getDriveControlLoop().correct(VecBuilder.fill(m_speeds.get().leftMetersPerSecond, m_speeds.get().rightMetersPerSecond));
            drivetrain.getDriveControlLoop().predict(dt);

            leftOutput = drivetrain.getDriveControlLoop().getU(0);
            rightOutput = drivetrain.getDriveControlLoop().getU(1);

            drivetrain.setVoltages(leftOutput, rightOutput);
        } else {
            double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

//          System.out.printf("%f, %f\n", (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond), dt);


            double rightFeedforward =
                    m_feedforward.calculate(rightSpeedSetpoint,
                            (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftMeasurement.setNumber(m_speeds.get().leftMetersPerSecond);
            leftReference.setNumber(leftSpeedSetpoint);

            rightMeasurement.setNumber(m_speeds.get().rightMetersPerSecond);
            rightReference.setNumber(rightSpeedSetpoint);

            drivetrain.setVelocities(leftSpeedSetpoint, leftFeedforward, rightSpeedSetpoint, rightFeedforward);
        }

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;

        LiveDashboardHelper.putRobotData(drivetrain.getCurrentPose());
        LiveDashboardHelper.putTrajectoryData(m_trajectory.sample(curTime).poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        LiveDashboard.getInstance().setFollowingPath(false);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}