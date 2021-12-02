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
                            new Pose2d(Units.feetToMeters(16.396), Units.feetToMeters(8.81), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackUpTwoFeetForShooterCalibration() {
            TrajectoryConfig config = new TrajectoryConfig(
                    sDrivetrain.getConfig().kMaxVelocityMetersPerSecond,
                    sDrivetrain.getConfig().kMaxAccelerationMetersPerSecondSquared);

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(20), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(22), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory sCurve = generateSCurve();

        public static Trajectory generateSCurve() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(8), Units.feetToMeters(4));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(13.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(10.5), Units.feetToMeters(18.5), Rotation2d.fromDegrees(0))
                    ),
                    config
            );
        }

    }

    public static class RoutineZero {

        public static Trajectory sForwards = generateForwards();
        public static Trajectory sBackwards = generateBackwards();

        public static Trajectory generateForwards() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(4), Units.feetToMeters(3));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(13.0),
                                    Rotation2d.fromDegrees(0.0)),
                            new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(13.0),
                                    Rotation2d.fromDegrees(0.0))),
                    config
            );
        }

        public static Trajectory generateBackwards() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(4), Units.feetToMeters(3));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(13.0),
                                    Rotation2d.fromDegrees(0.0)),
                            new Pose2d(Units.feetToMeters(10), Units.feetToMeters(13.0),
                                    Rotation2d.fromDegrees(0.0))),
                    config
            );
        }

    }

    public static class RoutineOne {

        public static Trajectory sPickupThreeFromTrench = generatePickupThreeFromTrench();
        public static Trajectory sBackupToShootThree = generateBackupToShootThree();

        public static Trajectory generatePickupThreeFromTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(8), Units.feetToMeters(4));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(19.146), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(16.502), Units.feetToMeters(21.54), Rotation2d.fromDegrees(35.345)),
                            new Pose2d(Units.feetToMeters(27.074), Units.feetToMeters(24.161), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackupToShootThree() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(8), Units.feetToMeters(4));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(27.074), Units.feetToMeters(24.161), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(18.198), Units.feetToMeters(22.365), Rotation2d.fromDegrees(32.381)),
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(19.146), Rotation2d.fromDegrees(0))),
                    config
            );
        }


    }

    public static class RoutineTwo {

        public static Trajectory sPickupTwoFromEnemyTrench = generatePickupTwoFromEnemyTrench();
        public static Trajectory sPickupTwoFromEnemyTrenchCurveIn = generatePickupTwoFromEnemyTrenchCurveIn();
        public static Trajectory sBackupToShootForSix = generateBackupToShootForSix();
        public static Trajectory sBackupToShootForFour = generateBackupToShootForFour();

        public static Trajectory generatePickupTwoFromEnemyTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generatePickupTwoFromEnemyTrenchCurveIn() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(4.0), Units.feetToMeters(2.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(19.364), Units.feetToMeters(2.357), Rotation2d.fromDegrees(20)),
                            new Pose2d(Units.feetToMeters(21.064), Units.feetToMeters(2.745), Rotation2d.fromDegrees(19.914))), config
            );
        }

        public static Trajectory generateBackupToShootForSix() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(8.0), Units.feetToMeters(4.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(21.064), Units.feetToMeters(2.745), Rotation2d.fromDegrees(19.914)),
                            new Pose2d(Units.feetToMeters(14.726), Units.feetToMeters(10.186), Rotation2d.fromDegrees(-79.119)),
                            new Pose2d(Units.feetToMeters(13.806), Units.feetToMeters(17.813), Rotation2d.fromDegrees(-31.14))),
                    config
            );
        }

        public static Trajectory generateBackupToShootForFour() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(8.0), Units.feetToMeters(4.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(21.064), Units.feetToMeters(2.745), Rotation2d.fromDegrees(19.914)),
                            new Pose2d(Units.feetToMeters(14.726), Units.feetToMeters(10.186), Rotation2d.fromDegrees(-79.119)),
                            new Pose2d(Units.feetToMeters(13.806), Units.feetToMeters(15.813), Rotation2d.fromDegrees(-50.0))),
                    config
            );
        }

    }

    public static class RoutineThree {

        public static Trajectory sPickupFiveFromTrench = generatePickupFiveFromTrench();
        public static Trajectory sBackupToShootFive = generateBackupToShootFive();

        public static Trajectory generatePickupFiveFromTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(19.146), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(18.585), Units.feetToMeters(22.958), Rotation2d.fromDegrees(32.381)),
                            new Pose2d(Units.feetToMeters(27.074), Units.feetToMeters(24.487), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(32), Units.feetToMeters(24.491), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackupToShootFive() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setReversed(true);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(32), Units.feetToMeters(24.491), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(27.074), Units.feetToMeters(24.487), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21.094), Units.feetToMeters(21.922), Rotation2d.fromDegrees(30)),
                            new Pose2d(Units.feetToMeters(16.25), Units.feetToMeters(19.146), Rotation2d.fromDegrees(30))),
                    config
            );
        }

    }

    public static class RoutineFour {

        public static Trajectory sBackupToAlignIntake = generateBackupToAlignIntake();
        public static Trajectory sGoIntoRendezvousForTwo = generateGoIntoRendezvousForTwo();
        public static Trajectory sBackupToShoot = generateBackupToShoot();

        public static Trajectory generateBackupToAlignIntake() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(27.074), Units.feetToMeters(24.487), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21.222), Units.feetToMeters(21.143), Rotation2d.fromDegrees(-70))),
                    config
            );
        }

        public static Trajectory generateGoIntoRendezvousForTwo() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(21.222), Units.feetToMeters(21.143), Rotation2d.fromDegrees(-70)),
                            new Pose2d(Units.feetToMeters(22.433), Units.feetToMeters(17.906), Rotation2d.fromDegrees(-70))),
                    config
            );
        }

        public static Trajectory generateBackupToShoot() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(22.433), Units.feetToMeters(17.906), Rotation2d.fromDegrees(-70)),
                            new Pose2d(Units.feetToMeters(16.283), Units.feetToMeters(19.77), Rotation2d.fromDegrees(30))),
                    config
            );
        }

    }

    public static class RoutineFive {

        // 5A
        public static Trajectory sPickupTwoInRendezvous = generatePickupTwoInRendezvous();
        public static Trajectory sBackupToShootFive = generateBackupToShootFive();

        // 5B
        public static Trajectory sPickupFourInRendezvous = generatePickupFourInRendezvous();
        public static Trajectory sBackupToShootFour = generateBackupToShootFour();

        public static Trajectory generatePickupTwoInRendezvous() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(13.948), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(22.74), Units.feetToMeters(15.278), Rotation2d.fromDegrees(20.215))),
                    config
            );
        }

        public static Trajectory generateBackupToShootFive() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(22.74), Units.feetToMeters(15.278), Rotation2d.fromDegrees(20.215)),
                            new Pose2d(Units.feetToMeters(16.302), Units.feetToMeters(14.523), Rotation2d.fromDegrees(-52.296)),
                            new Pose2d(Units.feetToMeters(13.803), Units.feetToMeters(17.287), Rotation2d.fromDegrees(-35))),
                    config
            );
        }

        public static Trajectory generatePickupFourInRendezvous() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(13.948), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21.661), Units.feetToMeters(13.487), Rotation2d.fromDegrees(27.412)),
                            new Pose2d(Units.feetToMeters(22.132), Units.feetToMeters(18.785), Rotation2d.fromDegrees(110))),
                    config
            );
        }

        public static Trajectory generateBackupToShootFour() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(22.132), Units.feetToMeters(18.785), Rotation2d.fromDegrees(110)),
                            new Pose2d(Units.feetToMeters(19.288), Units.feetToMeters(13.853), Rotation2d.fromDegrees(-0.61)),
                            new Pose2d(Units.feetToMeters(13.653), Units.feetToMeters(17.412), Rotation2d.fromDegrees(-35))),
                    config
            );
        }

    }

    public static class RoutineSix {

        public static Trajectory sPickupTwoInRendezvous = generatePickupTwoInRendezvous();

        public static Trajectory generatePickupTwoInRendezvous() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(13.744), Units.feetToMeters(16.8), Rotation2d.fromDegrees(-38.821)),
                            new Pose2d(Units.feetToMeters(22.74), Units.feetToMeters(15.278), Rotation2d.fromDegrees(20.215))),
                    config
            );
        }

    }

    public static class RoutineSeven {

        public static Trajectory sPickupTwoInEnemyTrench = generatePickupTwoInEnemyTrench();
        public static Trajectory sPickupExtraThreeInEnemyTrench = generatePickupExtraThreeInEnemyTrench();
        public static Trajectory sBackoutFromTrench = generateBackOutFromTrench();
        public static Trajectory sBackupToShootFive = generateBackupToShootFive();

        public static Trajectory generatePickupTwoInEnemyTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(6.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(11.671), Units.feetToMeters(12.848), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21.541), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generatePickupExtraThreeInEnemyTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setStartVelocity(6.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(21.541), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(32.216), Units.feetToMeters(2.081), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackOutFromTrench() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setEndVelocity(6.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(32.216), Units.feetToMeters(2.081), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(20.504), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateBackupToShootFive() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(6.0), Units.feetToMeters(3.0));

            config.setReversed(true);
            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.setStartVelocity(6.0);

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(20.504), Units.feetToMeters(2.312), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(13.803), Units.feetToMeters(16.287), Rotation2d.fromDegrees(-20))),
                    config
            );
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
                            kPowerPortReintroductionZonePose,
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
                            kPowerPortScoringZonePose,
                            kPowerPortReintroductionZonePose),
                    config
            );
        }

    }

}