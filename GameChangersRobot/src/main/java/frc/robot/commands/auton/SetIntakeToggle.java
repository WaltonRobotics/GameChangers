package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.time.Instant;

import static frc.robot.Robot.sIntake;

public class SetIntakeToggle extends SequentialCommandGroup {

    public SetIntakeToggle(boolean state, double waitSeconds) {
        addRequirements(sIntake);

        addCommands(new InstantCommand(() -> sIntake.setDeployed(true)));
        addCommands(new InstantCommand(() -> sIntake.setRetracted(false)));
        addCommands(new WaitCommand(waitSeconds));
        addCommands(new InstantCommand(() -> sIntake.setDeployed(false)));
    }

}
