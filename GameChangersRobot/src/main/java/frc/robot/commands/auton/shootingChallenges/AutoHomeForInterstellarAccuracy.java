package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import static frc.robot.Constants.Field.kMaximumInterstellarHomingRadius;
import static frc.robot.Constants.Field.kMinimumInterstellarHomingRadius;
import static frc.robot.Paths.ShootingChallengeRelativeHomingPaths.generateInterstellarAccuracyHomingTrajectory;
import static frc.robot.Paths.ShootingChallengeRelativeHomingPaths.sInterstellarHomingPose;
import static frc.robot.Robot.sDrivetrain;

public class AutoHomeForInterstellarAccuracy extends InstantCommand {

    @Override
    public void initialize() {
        Pose2d robotPose = sDrivetrain.getCurrentPose();
        double distanceToFinalPose = robotPose.getTranslation().getDistance(sInterstellarHomingPose.getTranslation());

        if (robotPose.getX() < sInterstellarHomingPose.getX()
                && distanceToFinalPose >= kMinimumInterstellarHomingRadius
                && distanceToFinalPose <= kMaximumInterstellarHomingRadius) {
            new RamseteTrackingCommand(
                    generateInterstellarAccuracyHomingTrajectory(),
                    true, false
            ).schedule();
        } else {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Failed to auto home");
        }
    }

}
