package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.RamseteTrackingCommand;

import static frc.robot.Paths.InterstellarAccuracyRelativePaths.generateShootingZoneOneTrajectory;

public class AutoHomeForInterstellarAccuracy extends CommandBase {

    @Override
    public void initialize() {
        new RamseteTrackingCommand(
                generateShootingZoneOneTrajectory(),
                true, false
        ).schedule();
    }

}
