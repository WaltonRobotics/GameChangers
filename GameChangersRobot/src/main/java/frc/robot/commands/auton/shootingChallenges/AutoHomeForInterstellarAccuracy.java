package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.HomingSupplier;

import java.util.logging.Level;

import static frc.robot.Constants.Field.*;
import static frc.robot.Paths.ShootingChallengeRelativeHomingPaths.generateInterstellarAccuracyHomingTrajectory;

public class AutoHomeForInterstellarAccuracy extends SequentialCommandGroup {

    public AutoHomeForInterstellarAccuracy() {
        addCommands(
                new ConditionalCommand(
                        new RamseteTrackingCommand(
                                generateInterstellarAccuracyHomingTrajectory(),
                                true, false
                        ),
                        new InstantCommand(() -> DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                                "Failed to auto home")),
                        new HomingSupplier(kInterstellarHomingPose, kMinimumInterstellarHomingRadius,
                                kMaximumInterstellarHomingRadius)
                )
        );
    }

}
