package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.sDrivetrain;

public class Paths {

        public static class TestTrajectory {
                public static Trajectory sTestTrajectory = generateTestTrajectory();

                public static Trajectory generateTestTrajectory() {
                        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(4.0),
                                        Units.feetToMeters(3.0));

                        config.setKinematics(sDrivetrain.getDriveKinematics());
                        config.addConstraint(new DifferentialDriveVoltageConstraint(sDrivetrain.getLinearFeedforward(),
                                        sDrivetrain.getDriveKinematics(), 10.0));

                        return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                        new Pose2d(Units.feetToMeters(7.541), Units.feetToMeters(4.866),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(20.145), Units.feetToMeters(12.491),
                                                        Rotation2d.fromDegrees(0))),
                                        config);
                }
        }

        public static class GalacticSearchPaths {
                public static Trajectory sRedA = generateGalacticSearchRedA();
                public static Trajectory sRedB = generateGalacticSearchRedB();
                public static Trajectory sBlueA = generateGalacticSearchBlueA();
                public static Trajectory sBlueB = generateGalacticSearchBlueB();

                public static Trajectory generateGalacticSearchRedA() {
                        TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                        config.setKinematics(sDrivetrain.getDriveKinematics());
                        config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                        return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                        new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(8.036), Units.feetToMeters(7.244),
                                                        Rotation2d.fromDegrees(-35.148)),
                                        new Pose2d(Units.feetToMeters(12.532), Units.feetToMeters(5.04),
                                                        Rotation2d.fromDegrees(20.55)),
                                        new Pose2d(Units.feetToMeters(15.223), Units.feetToMeters(12.669),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(26.43), Units.feetToMeters(13.525),
                                                        Rotation2d.fromDegrees(0))),
                                        config);
                }

                public static Trajectory generateGalacticSearchBlueA() {
                        TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                        config.setKinematics(sDrivetrain.getDriveKinematics());
                        config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                        return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                        new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(14.997), Units.feetToMeters(2.488),
                                                        Rotation2d.fromDegrees(13.634)),
                                        new Pose2d(Units.feetToMeters(16.74), Units.feetToMeters(6.26),
                                                        Rotation2d.fromDegrees(69.795)),
                                        new Pose2d(Units.feetToMeters(17.514), Units.feetToMeters(10.024),
                                                        Rotation2d.fromDegrees(-2.525)),
                                        new Pose2d(Units.feetToMeters(22.453), Units.feetToMeters(7.529),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(26.479), Units.feetToMeters(7.445),
                                                        Rotation2d.fromDegrees(0))),
                                        config);
                }

                public static Trajectory generateGalacticSearchRedB() {
                        TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                        config.setKinematics(sDrivetrain.getDriveKinematics());
                        config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                        return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                        new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(10),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(7.46), Units.feetToMeters(9.979),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(12.476), Units.feetToMeters(4.991),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(17.554), Units.feetToMeters(10.024),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(26.44), Units.feetToMeters(10.024),
                                                        Rotation2d.fromDegrees(0))),
                                        config);
                }

                public static Trajectory generateGalacticSearchBlueB() {
                        TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                        config.setKinematics(sDrivetrain.getDriveKinematics());
                        config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                        return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                        new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(5),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(15.077), Units.feetToMeters(5.004),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(19.993), Units.feetToMeters(10.003),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(25.012), Units.feetToMeters(4.95),
                                                        Rotation2d.fromDegrees(0)),
                                        new Pose2d(Units.feetToMeters(30.744), Units.feetToMeters(5.012),
                                                        Rotation2d.fromDegrees(0))),
                                        config);
                }

        }

        public static class AutonavPaths {
                
                public static class BouncePath {

                        public static Trajectory generateBounce1() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);


                                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                                new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5),
                                                                Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(7.44), Units.feetToMeters(10.89),
                                                                Rotation2d.fromDegrees(90))),
                                                config);
                        }

                        public static Trajectory generateBounce2() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);


                                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                                new Pose2d(Units.feetToMeters(7.44), Units.feetToMeters(10.89),
                                                                Rotation2d.fromDegrees(90)),
                                                new Pose2d(Units.feetToMeters(12.26), Units.feetToMeters(3.049),
                                                                Rotation2d.fromDegrees(180))),
                                                config);
                        }

                        public static Trajectory generateBounce3() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);


                                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                                new Pose2d(Units.feetToMeters(12.26), Units.feetToMeters(3.049),
                                                                Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(14.998), Units.feetToMeters(10.89),
                                                                Rotation2d.fromDegrees(270))),
                                                config);
                        }

                        public static Trajectory generateBounce4() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);


                                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                                                new Pose2d(Units.feetToMeters(14.998), Units.feetToMeters(10.89),
                                                                Rotation2d.fromDegrees(270)),
                                                new Pose2d(Units.feetToMeters(17.397), Units.feetToMeters(3.215),
                                                                Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(20.688), Units.feetToMeters(3.161),
                                                                Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(22.493), Units.feetToMeters(10.919),
                                                                Rotation2d.fromDegrees(90)),
                                                new Pose2d(Units.feetToMeters(26.42), Units.feetToMeters(7.445),
                                                                Rotation2d.fromDegrees(0))),
                                                config);
                        }
                }

                public static class SlalomPath {
                        public static Trajectory sSlalom = generateSlalom();
                        public static Trajectory generateSlalom() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                                return TrajectoryGenerator.generateTrajectory(
                                        Arrays.asList(
                                                new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(2.5), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(11.048), Units.feetToMeters(7.3), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(19.379), Units.feetToMeters(7.362), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(25.17), Units.feetToMeters(2.579), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(26.936), Units.feetToMeters(7.3), Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(23.207), Units.feetToMeters(6.302), Rotation2d.fromDegrees(-136)),
                                                new Pose2d(Units.feetToMeters(17.772), Units.feetToMeters(2.558), Rotation2d.fromDegrees(179.907)),
                                                new Pose2d(Units.feetToMeters(9.6), Units.feetToMeters(2.662), Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(4.899), Units.feetToMeters(6.343), Rotation2d.fromDegrees(180))),
                                        config
                                );
                        }
                        

                }

                public static class BarrelRacingPath {
                        public static Trajectory generateBarrelRacing() {
                                TrajectoryConfig config = new TrajectoryConfig(
                                        sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                                        sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);
                        
                                config.setKinematics(sDrivetrain.getDriveKinematics());
                                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);     
                        
                                return TrajectoryGenerator.generateTrajectory(
                                        Arrays.asList(
                                                new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(13.193), Units.feetToMeters(6.959), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(12.674), Units.feetToMeters(2.454), Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(9.441), Units.feetToMeters(5.407), Rotation2d.fromDegrees(90)),
                                                new Pose2d(Units.feetToMeters(19.993), Units.feetToMeters(8.069), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(20.787), Units.feetToMeters(12.187), Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(17.296), Units.feetToMeters(10.087), Rotation2d.fromDegrees(270)),
                                                new Pose2d(Units.feetToMeters(25.21), Units.feetToMeters(2.87), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Units.feetToMeters(25.944), Units.feetToMeters(7.5), Rotation2d.fromDegrees(180)),
                                                new Pose2d(Units.feetToMeters(3.61), Units.feetToMeters(8.194), Rotation2d.fromDegrees(180))),
                                        config
                                );
                        }
                }

        }

}