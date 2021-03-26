package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoAlign extends InstantCommand {

    @Override
    public void initialize() {
//        new TurnToAngle(sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
//                .beforeStarting(new SequentialCommandGroup(
//                        new InstantCommand(() -> LimelightHelper.setLEDMode(true))
//                ))
//                .andThen(() -> LimelightHelper.setLEDMode(false)).schedule();
    }

}
