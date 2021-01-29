package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.drivetrain;

public class Paths {

    public static Trajectory generateGalacticSearchRedA() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(10.0), Units.feetToMeters(4.0));

        config.setKinematics(drivetrain.getDriveKinematics());
        config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(Units.feetToMeters(2.659), Units.feetToMeters(7.438), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(7.52), Units.feetToMeters(7.458), Rotation2d.fromDegrees(-0.063)),
                        new Pose2d(Units.feetToMeters(9.084), Units.feetToMeters(6.801), Rotation2d.fromDegrees(-11.616)),
                        new Pose2d(Units.feetToMeters(10.612), Units.feetToMeters(5.865), Rotation2d.fromDegrees(-35.011)),
                        new Pose2d(Units.feetToMeters(12.456), Units.feetToMeters(4.908), Rotation2d.fromDegrees(-2.497)),
                        new Pose2d(Units.feetToMeters(13.706), Units.feetToMeters(5.74), Rotation2d.fromDegrees(46.665)),
                        new Pose2d(Units.feetToMeters(14.102), Units.feetToMeters(7.549), Rotation2d.fromDegrees(86.205)),
                        new Pose2d(Units.feetToMeters(13.329), Units.feetToMeters(9.608), Rotation2d.fromDegrees(75.206)),
                        new Pose2d(Units.feetToMeters(12.992), Units.feetToMeters(11.584), Rotation2d.fromDegrees(46.546)),
                        new Pose2d(Units.feetToMeters(15.074), Units.feetToMeters(12.478), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(25.646), Units.feetToMeters(12.478), Rotation2d.fromDegrees(0))),
                config
        );
    }

}
