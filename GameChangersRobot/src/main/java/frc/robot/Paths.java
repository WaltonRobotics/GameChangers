package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.sDrivetrain;

public class Paths {

    public static class TestTrajectory {
        public static Trajectory testTrajectory = generateTestTrajectory();

        public static Trajectory generateTestTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(4.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(3.28084), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0))),
                    config
            );
        }
    }

    public static class GalacticSearchPaths {
        public static Trajectory redA = generateGalacticSearchRedA();
        public static Trajectory redB = generateGalacticSearchRedB();
        public static Trajectory blueA = generateGalacticSearchBlueA();
        public static Trajectory blueB = generateGalacticSearchBlueB();

        public static Trajectory generateGalacticSearchRedA() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(4.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(4.042), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(8.036), Units.feetToMeters(7.244), Rotation2d.fromDegrees(-35.148)),
                            new Pose2d(Units.feetToMeters(12.532), Units.feetToMeters(5.04), Rotation2d.fromDegrees(20.55)),
                            new Pose2d(Units.feetToMeters(15.223), Units.feetToMeters(12.669), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(24.959), Units.feetToMeters(12.461), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueA() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(4.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(4.042), Units.feetToMeters(2.529), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(15.017), Units.feetToMeters(2.55), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(17.911), Units.feetToMeters(3.182), Rotation2d.fromDegrees(30.639)),
                            new Pose2d(Units.feetToMeters(18.486), Units.feetToMeters(4.596), Rotation2d.fromDegrees(61.051)),
                            new Pose2d(Units.feetToMeters(18.863), Units.feetToMeters(6.53), Rotation2d.fromDegrees(86.903)),
                            new Pose2d(Units.feetToMeters(18.466), Units.feetToMeters(8.111), Rotation2d.fromDegrees(106.869)),
                            new Pose2d(Units.feetToMeters(17.474), Units.feetToMeters(10.003), Rotation2d.fromDegrees(90.008)),
                            new Pose2d(Units.feetToMeters(17.732), Units.feetToMeters(11.646), Rotation2d.fromDegrees(34.879)),
                            new Pose2d(Units.feetToMeters(18.883), Units.feetToMeters(12.458), Rotation2d.fromDegrees(12.894)),
                            new Pose2d(Units.feetToMeters(20.608), Units.feetToMeters(12.603), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(21.759), Units.feetToMeters(11.834), Rotation2d.fromDegrees(-35.847)),
                            new Pose2d(Units.feetToMeters(22.453), Units.feetToMeters(10.211), Rotation2d.fromDegrees(-74.238)),
                            new Pose2d(Units.feetToMeters(22.512), Units.feetToMeters(7.612), Rotation2d.fromDegrees(-88.645)),
                            new Pose2d(Units.feetToMeters(22.79), Units.feetToMeters(5.698), Rotation2d.fromDegrees(-60.453)),
                            new Pose2d(Units.feetToMeters(24.417), Units.feetToMeters(4.035), Rotation2d.fromDegrees(-31.355)),
                            new Pose2d(Units.feetToMeters(26.896), Units.feetToMeters(3.723), Rotation2d.fromDegrees(-3.769))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchRedB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(4.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(2.54), Units.feetToMeters(10.016), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(7.48), Units.feetToMeters(10.016), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(9.54), Units.feetToMeters(9.692), Rotation2d.fromDegrees(-15.24)),
                            new Pose2d(Units.feetToMeters(10.77), Units.feetToMeters(8.964), Rotation2d.fromDegrees(-36.395)),
                            new Pose2d(Units.feetToMeters(11.861), Units.feetToMeters(7.591), Rotation2d.fromDegrees(-56.997)),
                            new Pose2d(Units.feetToMeters(12.357), Units.feetToMeters(6.156), Rotation2d.fromDegrees(-69.565)),
                            new Pose2d(Units.feetToMeters(12.496), Units.feetToMeters(4.991), Rotation2d.fromDegrees(-88.125)),
                            new Pose2d(Units.feetToMeters(12.972), Units.feetToMeters(3.515), Rotation2d.fromDegrees(-73.556)),
                            new Pose2d(Units.feetToMeters(13.686), Units.feetToMeters(2.516), Rotation2d.fromDegrees(-30.378)),
                            new Pose2d(Units.feetToMeters(14.936), Units.feetToMeters(1.685), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(16.641), Units.feetToMeters(2.038), Rotation2d.fromDegrees(18.114)),
                            new Pose2d(Units.feetToMeters(17.732), Units.feetToMeters(3.369), Rotation2d.fromDegrees(53.946)),
                            new Pose2d(Units.feetToMeters(18.347), Units.feetToMeters(5.054), Rotation2d.fromDegrees(75.471)),
                            new Pose2d(Units.feetToMeters(18.347), Units.feetToMeters(7.196), Rotation2d.fromDegrees(96.967)),
                            new Pose2d(Units.feetToMeters(17.653), Units.feetToMeters(8.922), Rotation2d.fromDegrees(114.496)),
                            new Pose2d(Units.feetToMeters(17.514), Units.feetToMeters(9.983), Rotation2d.fromDegrees(90.47)),
                            new Pose2d(Units.feetToMeters(18.05), Units.feetToMeters(11.667), Rotation2d.fromDegrees(52.369)),
                            new Pose2d(Units.feetToMeters(19.339), Units.feetToMeters(12.478), Rotation2d.fromDegrees(18.032)),
                            new Pose2d(Units.feetToMeters(20.886), Units.feetToMeters(12.603), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(26.975), Units.feetToMeters(12.562), Rotation2d.fromDegrees(0))),
                    config
            );
        }

        public static Trajectory generateGalacticSearchBlueB() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(10.0), Units.feetToMeters(4.0));

            config.setKinematics(sDrivetrain.getDriveKinematics());
            config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(4.0)));

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(5.046), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(15.077), Units.feetToMeters(5.004), Rotation2d.fromDegrees(0)),
                            new Pose2d(Units.feetToMeters(17.455), Units.feetToMeters(5.095), Rotation2d.fromDegrees(22.43)),
                            new Pose2d(Units.feetToMeters(19.101), Units.feetToMeters(6.364), Rotation2d.fromDegrees(57.822)),
                            new Pose2d(Units.feetToMeters(20.013), Units.feetToMeters(8.652), Rotation2d.fromDegrees(89.876)),
                            new Pose2d(Units.feetToMeters(20.033), Units.feetToMeters(10.024), Rotation2d.fromDegrees(89.609)),
                            new Pose2d(Units.feetToMeters(20.569), Units.feetToMeters(11.168), Rotation2d.fromDegrees(44.336)),
                            new Pose2d(Units.feetToMeters(21.56), Units.feetToMeters(11.938), Rotation2d.fromDegrees(18.172)),
                            new Pose2d(Units.feetToMeters(22.929), Units.feetToMeters(12.042), Rotation2d.fromDegrees(-0.422)),
                            new Pose2d(Units.feetToMeters(24.04), Units.feetToMeters(11.438), Rotation2d.fromDegrees(-49.293)),
                            new Pose2d(Units.feetToMeters(25.031), Units.feetToMeters(9.65), Rotation2d.fromDegrees(-85.205)),
                            new Pose2d(Units.feetToMeters(24.992), Units.feetToMeters(4.991), Rotation2d.fromDegrees(-90.375)),
                            new Pose2d(Units.feetToMeters(25.309), Units.feetToMeters(3.494), Rotation2d.fromDegrees(-65.051)),
                            new Pose2d(Units.feetToMeters(26.122), Units.feetToMeters(2.6), Rotation2d.fromDegrees(-31.179)),
                            new Pose2d(Units.feetToMeters(27.669), Units.feetToMeters(2.267), Rotation2d.fromDegrees(0))),
                    config
            );
        }
    }

    public static Trajectory generateBarrelRacingPath() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(4.0), Units.feetToMeters(3.0));

        config.setKinematics(sDrivetrain.getDriveKinematics());
        config.addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(3.0)));

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

}