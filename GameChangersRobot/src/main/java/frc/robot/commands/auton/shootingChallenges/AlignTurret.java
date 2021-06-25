package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auton.AutonFlags;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Turret.kAlignedThresholdDegrees;
import static frc.robot.Constants.Turret.kAlignmentTimeout;

public class AlignTurret extends SequentialCommandGroup {

    public AlignTurret() {
        addCommands(
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToAlignTurret(true)),
                new WaitUntilCommand(() -> LimelightHelper.getTX() < kAlignedThresholdDegrees)
                        .withTimeout(kAlignmentTimeout),
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToAlignTurret(false))
        );
    }

}
