package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Robot.sDrivetrain;

public class AutoAlign extends InstantCommand {

    @Override
    public void initialize() {
        new TurnToAngle(sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
                .beforeStarting(() -> LimelightHelper.setLEDMode(true))
                .andThen(() -> LimelightHelper.setLEDMode(false)).schedule();
    }

}
