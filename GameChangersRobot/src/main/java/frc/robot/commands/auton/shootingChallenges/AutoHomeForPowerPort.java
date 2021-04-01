package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
public class AutoHomeForPowerPort extends SequentialCommandGroup {

    public AutoHomeForPowerPort() {
        addCommands(
                new AutomatedTrackingCommand(
                        Paths.ShootingChallengeRelativePaths::generatePowerPortHomingTrajectory,
                        true, false
                )
        );
    }

}
