package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auton.RamseteTrackingCommand;

import static frc.robot.Paths.ShootingChallengeRelativeHomingPaths.generateInterstellarAccuracyHomingTrajectory;

public class AutoHomeForInterstellarAccuracy extends InstantCommand {

    @Override
    public void initialize() {
        new RamseteTrackingCommand(
                generateInterstellarAccuracyHomingTrajectory(),
                true, false
        ).schedule();
    }

}
