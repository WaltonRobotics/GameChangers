package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;
import frc.robot.vision.PnPHelper;

import java.util.logging.Level;

import static frc.robot.Robot.sDrivetrain;

public class CalibratePose extends SequentialCommandGroup {

    public CalibratePose() {
        addCommands(
                new InstantCommand(this::calibratePose)
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
