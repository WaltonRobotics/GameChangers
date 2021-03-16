package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.sCurrentRobot;
import static frc.robot.Robot.sDrivetrain;

public class Paths {

    public static Trajectory generateBarrelRacingPath() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(10.0), Units.feetToMeters(6.0));

        config.setKinematics(sDrivetrain.getDriveKinematics());
        config.addConstraint(
                new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
                        sDrivetrain.getDriveKinematics(), 10.0));

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0.069)),
                        new Pose2d(Units.feetToMeters(13.895), Units.feetToMeters(6.116), Rotation2d.fromDegrees(-94.485)),
                        new Pose2d(Units.feetToMeters(12.186), Units.feetToMeters(2.624), Rotation2d.fromDegrees(111.26)),
                        new Pose2d(Units.feetToMeters(11.692), Units.feetToMeters(6.34), Rotation2d.fromDegrees(39.031)),
                        new Pose2d(Units.feetToMeters(21.574), Units.feetToMeters(8.846), Rotation2d.fromDegrees(59.526)),
                        new Pose2d(Units.feetToMeters(19.21), Units.feetToMeters(11.928), Rotation2d.fromDegrees(-136.805)),
                        new Pose2d(Units.feetToMeters(19.336), Units.feetToMeters(7.259), Rotation2d.fromDegrees(-28.706)),
                        new Pose2d(Units.feetToMeters(25.978), Units.feetToMeters(3.025), Rotation2d.fromDegrees(27.306)),
                        new Pose2d(Units.feetToMeters(25.458), Units.feetToMeters(7.656), Rotation2d.fromDegrees(178.519)),
                        new Pose2d(Units.feetToMeters(2.46), Units.feetToMeters(8.28), Rotation2d.fromDegrees(-179.633))),
                config
        );
    }

    public static class TestTrajectory {
        public static Trajectory sTestTrajectory = generateTestTrajectory();

        public static Trajectory generateTestTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(4.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(7.541), Units.feetToMeters(4.866), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(20.145), Units.feetToMeters(12.491), Rotation2d.fromDegrees(0))),
                    config
            );
        }
    }

    public static class GalacticSearchPaths {
        public static Trajectory sRedA = generateGalacticSearchRedA();
        public static Trajectory sRedB = generateGalacticSearchRedB();
        public static Trajectory sBlueA = generateGalacticSearchBlueA();
        public static Trajectory sBlueB = generateGalacticSearchBlueB();

        public static Trajectory generateGalacticSearchRedA() {


            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(sCurrentRobot.getCurrentRobot().getDrivetrainMaxVelocity()),
                    Units.feetToMeters(sCurrentRobot.getCurrentRobot().getDrivetrainMaxAcceleration()));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(sCurrentRobot.getCurrentRobot().getDrivetrainMaxVelocity());
//            config.addConstraint(
//                    new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
//                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.729), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(8.036), Units.feetToMeters(7.244), Rotation2d.fromDegrees(-35.148)),
                            new Pose2d(Units.feetToMeters(12.532), Units.feetToMeters(5.04), Rotation2d.fromDegrees(20.55)),
                            new Pose2d(Units.feetToMeters(15.223), Units.feetToMeters(12.669), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(26.43), Units.feetToMeters(13.525), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueA() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(6.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(4.042), Units.feetToMeters(2.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(14.997), Units.feetToMeters(2.488), Rotation2d.fromDegrees(13.634)),
                            new Pose2d(Units.feetToMeters(16.74), Units.feetToMeters(6.26), Rotation2d.fromDegrees(69.795)),
                            new Pose2d(Units.feetToMeters(17.514), Units.feetToMeters(10.024), Rotation2d.fromDegrees(-2.525)),
                            new Pose2d(Units.feetToMeters(22.453), Units.feetToMeters(7.529), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(26.479), Units.feetToMeters(7.445), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchRedB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(6.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(4.042), Units.feetToMeters(10), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(7.46), Units.feetToMeters(9.979), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(12.476), Units.feetToMeters(4.991), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(17.554), Units.feetToMeters(10.024), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(26.44), Units.feetToMeters(10.024), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(6.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(sDrivetrain.getFeedforward(),
                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(15.077), Units.feetToMeters(5.004), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(19.993), Units.feetToMeters(10.003), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(25.012), Units.feetToMeters(4.95), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(30.744), Units.feetToMeters(5.012), Rotation2d.fromDegrees(0))),
                    config
            );
        }

    }

}