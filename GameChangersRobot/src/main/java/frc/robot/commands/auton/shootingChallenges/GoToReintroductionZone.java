package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import static frc.robot.Constants.Field.kPowerPortReintroductionZonePose;
import static frc.robot.Robot.sDrivetrain;

public class GoToReintroductionZone extends SequentialCommandGroup {

    public GoToReintroductionZone() {
        addCommands(
                new ConditionalCommand(
                        new ManualOverrideTrackingCommand(
                                Paths.ShootingChallengeRelativePaths::generatePowerPortToReintroductionZoneTrajectory,
                                true, false
                        ),
                        new InstantCommand(() -> DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                                "Failed to go to reintroduction zone")),
                        () -> sDrivetrain.getCurrentPose().getX() > kPowerPortReintroductionZonePose.getX()
                )
        );
    }

}
