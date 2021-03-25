package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.vision.LimelightHelper;
import frc.robot.vision.PnPHelper;

import static frc.robot.Constants.Shooter.kLimelightLEDWaitTimeSeconds;
import static frc.robot.Robot.sDrivetrain;

public class CalibratePose extends SequentialCommandGroup {

    public CalibratePose() {
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new WaitCommand(kLimelightLEDWaitTimeSeconds),
                new InstantCommand(this::calibratePose),
                new InstantCommand(() -> LimelightHelper.setLEDMode(false))
        );
    }

    private void calibratePose() {
        Pose2d currentPose = PnPHelper.getEstimatedPose();
        sDrivetrain.resetPose(currentPose);
    }

}
