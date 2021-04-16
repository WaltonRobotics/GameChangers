package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.AutonFlags;

import static frc.robot.Constants.Turret.kAlignmentTimeout;

public class AlignTurret extends SequentialCommandGroup {

    public AlignTurret(double timeout) {
        addCommands(
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToAlignTurret(true)),
                new WaitCommand(timeout),
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToAlignTurret(false))
        );
    }

    public AlignTurret() {
        this(kAlignmentTimeout);
    }

}
