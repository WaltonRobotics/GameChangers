package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;
import frc.robot.vision.PnPHelper;

import java.util.logging.Level;

import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import static frc.robot.Robot.sDrivetrain;

public class CalibratePose extends SequentialCommandGroup {

    public CalibratePose() {
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new WaitUntilCommand(() -> LimelightHelper.getTV() > 0).withTimeout(kMaximumLEDWaitTimeSeconds),
                new InstantCommand(this::calibratePose),
                new InstantCommand(() -> LimelightHelper.setLEDMode(false))
        );
    }

    private void calibratePose() {
        if (LimelightHelper.getTV() <= 0) {
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "No target found to calibrate pose. Using last known information");
        }

        Pose2d currentPose = PnPHelper.getEstimatedPose();
        sDrivetrain.resetPose(currentPose);
    }

}
