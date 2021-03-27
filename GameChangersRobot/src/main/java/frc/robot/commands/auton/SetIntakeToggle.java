package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Robot.sIntake;

public class SetIntakeToggle extends SequentialCommandGroup {

    public SetIntakeToggle(boolean state) {
        if (state) {
            addCommands(
                    new InstantCommand(() -> sIntake.setDeployed(true)),
                    new InstantCommand(() -> sIntake.setRetracted(false)),
                    new WaitCommand(sIntake.getConfig().kSettleTime),
                    new InstantCommand(() -> sIntake.setDeployed(false)),
                    new InstantCommand(() -> sIntake.setRetracted(false))
            );
        } else {
            addCommands(
                    new InstantCommand(() -> sIntake.setDeployed(false)),
                    new InstantCommand(() -> sIntake.setRetracted(true))

            );
        }
    }

}
