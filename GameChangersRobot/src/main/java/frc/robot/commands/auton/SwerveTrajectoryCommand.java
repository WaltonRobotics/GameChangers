package frc.robot.commands.auton;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTrajectoryCommand extends CommandBase {
    private final SwerveDrivetrain swerveDriveTrain = Robot.sSwerveDriveTrain;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();
    private HolonomicDriveController holonomicDriveController;
    private final Pose2d odometryPose = new Pose2d();

    public SwerveTrajectoryCommand(Trajectory trajectory) {
        addRequirements(swerveDriveTrain);
        this.trajectory = trajectory;

    }

    public void initialize() {  //dummy values
        var p = 6.0;
        var d = p / 100.0;
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        2.5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(Constants.SwerveDrive.kMaxOmega / 2.0, 3.14));
        thetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        holonomicDriveController =
                new HolonomicDriveController(
                        new PIDController(p, 0, d), new PIDController(p, 0, d), thetaController);

        holonomicDriveController.setEnabled(true);
        swerveDriveTrain.resetOdometry(trajectory.getInitialPose());    //where are these trajectory methods
        timer.reset();
    }

    public void execute() {
        Trajectory.State state = trajectory.sample(timer.get());
        ChassisSpeeds speeds = holonomicDriveController.calculate(odometryPose, state, Rotation2d.fromDegrees(180));
        swerveDriveTrain.move(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDrivetrain.drive(0.0, 0.0, 0.0);
    }
}
