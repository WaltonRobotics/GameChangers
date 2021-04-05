package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Constants.Field.*;
import static frc.robot.Robot.sDrivetrain;

public class Paths {

    public static class MiscellaneousTrajectories {
        public static Trajectory sTestTrajectory = generateTestTrajectory();
        public static Trajectory sShooterCalibrationTrajectory = generateBackUpTwoFeetForShooterCalibration();

        public static Trajectory generateTestTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(4.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(sDrivetrain.getLinearFeedforward(),
                            sDrivetrain.getDriveKinematics(), 10.0));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(7.541), Units.feetToMeters(4.866), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(20.145), Units.feetToMeters(12.491), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackUpTwoFeetForShooterCalibration() {
            TrajectoryConfig config = new TrajectoryConfig(
                    sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                    sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(22), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(20), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0))),
                    config
            );
        }

    }

    public static class GalacticSearchPaths {
        public static Trajectory sRedATrajectory = generateGalacticSearchRedA();
        public static Trajectory sRedBTrajectory = generateGalacticSearchRedB();
        public static Trajectory sBlueATrajectory = generateGalacticSearchBlueA();
        public static Trajectory sBlueBTrajectory = generateGalacticSearchBlueB();

        public static Trajectory generateGalacticSearchRedA() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.0,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.771), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(8.691), Units.feetToMeters(6.724), Rotation2d.fromDegrees(-43.374)),
                            new Pose2d(Units.feetToMeters(12.195), Units.feetToMeters(4.915), Rotation2d.fromDegrees(-1.673)),
                            new Pose2d(Units.feetToMeters(14.904), Units.feetToMeters(8.234), Rotation2d.fromDegrees(89.562)),
                            new Pose2d(Units.feetToMeters(14.841), Units.feetToMeters(12.634), Rotation2d.fromDegrees(90.618)),
                            new Pose2d(Units.feetToMeters(18.526), Units.feetToMeters(12.562), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueA() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.0,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.771), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(14.997), Units.feetToMeters(2.471), Rotation2d.fromDegrees(-26.226)),
                            new Pose2d(Units.feetToMeters(17.55), Units.feetToMeters(6.017), Rotation2d.fromDegrees(86.038)),
                            new Pose2d(Units.feetToMeters(17.712), Units.feetToMeters(10.215), Rotation2d.fromDegrees(83.095)),
                            new Pose2d(Units.feetToMeters(21.908), Units.feetToMeters(10.042), Rotation2d.fromDegrees(-61.479)),
                            new Pose2d(Units.feetToMeters(22.58), Units.feetToMeters(6.965), Rotation2d.fromDegrees(-88.759)),
                            new Pose2d(Units.feetToMeters(25.223), Units.feetToMeters(6.135), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchRedB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.0,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.771), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(7.619), Units.feetToMeters(10.145), Rotation2d.fromDegrees(44.03)),
                            new Pose2d(Units.feetToMeters(11.842), Units.feetToMeters(8.481), Rotation2d.fromDegrees(-79.243)),
                            new Pose2d(Units.feetToMeters(12.558), Units.feetToMeters(5.063), Rotation2d.fromDegrees(-90.497)),
                            new Pose2d(Units.feetToMeters(14.003), Units.feetToMeters(2.695), Rotation2d.fromDegrees(-34.674)),
                            new Pose2d(Units.feetToMeters(17.772), Units.feetToMeters(5.352), Rotation2d.fromDegrees(90.822)),
                            new Pose2d(Units.feetToMeters(17.851), Units.feetToMeters(10.295), Rotation2d.fromDegrees(76.063)),
                            new Pose2d(Units.feetToMeters(20.678), Units.feetToMeters(11.456), Rotation2d.fromDegrees(1.628))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.0,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.771), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(15.259), Units.feetToMeters(5.039), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(19.695), Units.feetToMeters(9.552), Rotation2d.fromDegrees(47.76)),
                            new Pose2d(Units.feetToMeters(23.316), Units.feetToMeters(9.315), Rotation2d.fromDegrees(-54.487)),
                            new Pose2d(Units.feetToMeters(24.912), Units.feetToMeters(5.3), Rotation2d.fromDegrees(-62.59)),
                            new Pose2d(Units.feetToMeters(26.793), Units.feetToMeters(3.31), Rotation2d.fromDegrees(0))),
                    config
            );
        }
    }

    public static class AutonavPaths {
        public static Trajectory sBarrelRacingTrajectory = generateBarrelRacing();
        public static Trajectory sSlalomTrajectory = generateSlalom();

        public static Trajectory generateBarrelRacing() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.5,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.5);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
			new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
			new Pose2d(Units.feetToMeters(13.567), Units.feetToMeters(7.34), Rotation2d.fromDegrees(-31.191)),
			new Pose2d(Units.feetToMeters(12.555), Units.feetToMeters(2.288), Rotation2d.fromDegrees(-177.29)),
			new Pose2d(Units.feetToMeters(9.454), Units.feetToMeters(6.474), Rotation2d.fromDegrees(46.489)),
			new Pose2d(Units.feetToMeters(20.727), Units.feetToMeters(7.784), Rotation2d.fromDegrees(31.708)),
			new Pose2d(Units.feetToMeters(20.37), Units.feetToMeters(13.566), Rotation2d.fromDegrees(-179.541)),
			new Pose2d(Units.feetToMeters(16.268), Units.feetToMeters(10.312), Rotation2d.fromDegrees(-66.318)),
			new Pose2d(Units.feetToMeters(25.008), Units.feetToMeters(2.177), Rotation2d.fromDegrees(13.186)),
			new Pose2d(Units.feetToMeters(25.811), Units.feetToMeters(7.943), Rotation2d.fromDegrees(179.339)),
			new Pose2d(Units.feetToMeters(8.767), Units.feetToMeters(8.575), Rotation2d.fromDegrees(180)),
			new Pose2d(Units.feetToMeters(3.157), Units.feetToMeters(8.579), Rotation2d.fromDegrees(-179.723))),
                    config
            );  
        }

        public static Trajectory generateSlalom() {
            TrajectoryConfig config = new TrajectoryConfig(
                    3.5,
                    0.8);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(3.5);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(2.448), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(8.284), Units.feetToMeters(6.289), Rotation2d.fromDegrees(37.109)),
                            new Pose2d(Units.feetToMeters(15.645), Units.feetToMeters(6.961), Rotation2d.fromDegrees(-1.792)),
                            new Pose2d(Units.feetToMeters(21.9), Units.feetToMeters(6.172), Rotation2d.fromDegrees(-40.15)),
                            new Pose2d(Units.feetToMeters(25.253), Units.feetToMeters(2.648), Rotation2d.fromDegrees(-22.846)),
                            new Pose2d(Units.feetToMeters(27.614), Units.feetToMeters(6.018), Rotation2d.fromDegrees(105.646)),
                            new Pose2d(Units.feetToMeters(23.554), Units.feetToMeters(6.614), Rotation2d.fromDegrees(-132.818)),
                            new Pose2d(Units.feetToMeters(20.698), Units.feetToMeters(2.506), Rotation2d.fromDegrees(-151.042)),
                            new Pose2d(Units.feetToMeters(12.955), Units.feetToMeters(2.402), Rotation2d.fromDegrees(178.15)),
                            new Pose2d(Units.feetToMeters(7.215), Units.feetToMeters(3.798), Rotation2d.fromDegrees(114.744)),
                            new Pose2d(Units.feetToMeters(3.15), Units.feetToMeters(7.836), Rotation2d.fromDegrees(179.202))),
                    config
            );
        }

        public static class BouncePaths {

            public static Trajectory sBounce1Trajectory = generateBounce1();
            public static Trajectory sBounce2Trajectory = generateBounce2();
            public static Trajectory sBounce3Trajectory = generateBounce3();
            public static Trajectory sBounce4Trajectory = generateBounce4();

            public static Trajectory generateBounce1() {
                TrajectoryConfig config = new TrajectoryConfig(
                        3.5,
                        0.8);

                config.setKinematics(sDrivetrain.getDriveKinematics());
//                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(Units.feetToMeters(3.146), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
			new Pose2d(Units.feetToMeters(7.44), Units.feetToMeters(12), Rotation2d.fromDegrees(90))),
                        config);
            }

            public static Trajectory generateBounce2() {
                TrajectoryConfig config = new TrajectoryConfig(
                        3.5,
                        0.8);

                config.setKinematics(sDrivetrain.getDriveKinematics());
//                config.setStartVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);
//                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);
                config.setReversed(true);

                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(Units.feetToMeters(7.44), Units.feetToMeters(12), Rotation2d.fromDegrees(90)),
                        new Pose2d(Units.feetToMeters(9.789), Units.feetToMeters(5.357), Rotation2d.fromDegrees(119.875)),
                        new Pose2d(Units.feetToMeters(13.387), Units.feetToMeters(2.606), Rotation2d.fromDegrees(-146.2)),
                        new Pose2d(Units.feetToMeters(15.005), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90))),
                        config);
            }

            public static Trajectory generateBounce3() {
                TrajectoryConfig config = new TrajectoryConfig(
                        3.5,
                        0.8);

                config.setKinematics(sDrivetrain.getDriveKinematics());
//                config.setStartVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);
//                config.setEndVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);

                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(Units.feetToMeters(15.005), Units.feetToMeters(12.5), Rotation2d.fromDegrees(-90)),
			new Pose2d(Units.feetToMeters(18.903), Units.feetToMeters(2.145), Rotation2d.fromDegrees(0)),
			new Pose2d(Units.feetToMeters(23.337), Units.feetToMeters(12), Rotation2d.fromDegrees(90))),
                        config);
            }

            public static Trajectory generateBounce4() {
                TrajectoryConfig config = new TrajectoryConfig(
                        3.5,
                        0.8);

                config.setKinematics(sDrivetrain.getDriveKinematics());
//                config.setStartVelocity(sDrivetrain.getConfig().kMaxVelocityMetersPerSecond);
                config.setEndVelocity(3.5);
                config.setReversed(true);

                return TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(Units.feetToMeters(23.337), Units.feetToMeters(12), Rotation2d.fromDegrees(90)),
			new Pose2d(Units.feetToMeters(25.199), Units.feetToMeters(7.086), Rotation2d.fromDegrees(179.856)),
			new Pose2d(Units.feetToMeters(28.978), Units.feetToMeters(5.727), Rotation2d.fromDegrees(180))),
                        config);
            }
        }
    }

    public static class ShootingChallengeRelativePaths {

        public static Trajectory generateInterstellarAccuracyHomingTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                    sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            sDrivetrain.getCurrentPose(),
                            kInterstellarHomingPose),
                    config
            );
        }

        public static Trajectory generatePowerPortToScoringZoneTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    6,
                    3);

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            sDrivetrain.getCurrentPose(),
                            kPowerPortScoringZonePose),
                    config
            );
        }

        public static Trajectory generatePowerPortToReintroductionZoneTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    6,
                    3);

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            sDrivetrain.getCurrentPose(),
                            kPowerPortReintroductionZonePose),
                    config
            );
        }

    }

}