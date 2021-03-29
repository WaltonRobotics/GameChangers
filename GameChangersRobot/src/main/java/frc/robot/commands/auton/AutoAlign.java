package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import static frc.robot.Robot.sDrivetrain;

public class AutoAlign extends SequentialCommandGroup {

    public AutoAlign() {
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new InstantCommand(() -> LimelightHelper.setPipeline(kAlignmentPipeline)),
                new WaitUntilCommand(() -> LimelightHelper.getTV() > 0).withTimeout(kMaximumLEDWaitTimeSeconds),
                new TurnToAngle(() -> sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
//                new InstantCommand(() -> LimelightHelper.setLEDMode(false))
        );
    }

}
