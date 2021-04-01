package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;

public class GoToReintroductionZone extends SequentialCommandGroup {

    public GoToReintroductionZone() {
        addCommands(
                new AutomatedTrackingCommand(
                        Paths.ShootingChallengeRelativePaths::generatePowerPortToReintroductionZoneTrajectory,
                        true, false
                )
        );
    }

}
