package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Robot.sDrivetrain;

public class AutoAlign extends SequentialCommandGroup {

    public AutoAlign() {
        addCommands(
                new TurnToAngle(() -> sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
        );
    }

}
