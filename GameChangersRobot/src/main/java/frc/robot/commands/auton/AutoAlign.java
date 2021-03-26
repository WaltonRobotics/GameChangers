package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Shooter.kLimelightLEDWaitTimeSeconds;
import static frc.robot.Robot.sDrivetrain;

public class AutoAlign extends SequentialCommandGroup {

    public AutoAlign() {
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new WaitCommand(kLimelightLEDWaitTimeSeconds),
                new TurnToAngle(() -> sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX()),
                new InstantCommand(() -> LimelightHelper.setLEDMode(false))
        );
    }

}
