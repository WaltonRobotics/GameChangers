package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.shootingChallenges.ManualOverrideTrackingCommand;

import static frc.robot.Paths.MiscellaneousTrajectories.sShooterCalibrationTrajectory;

public class BackUpTwoFeetForShooterCalibration extends SequentialCommandGroup {

    public BackUpTwoFeetForShooterCalibration() {
        addCommands(
                new ResetPose(sShooterCalibrationTrajectory),
                new ManualOverrideTrackingCommand(sShooterCalibrationTrajectory, true, false)
        );
    }

}
