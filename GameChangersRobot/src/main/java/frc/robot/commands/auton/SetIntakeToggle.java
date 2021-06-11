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
                    new WaitCommand(sIntake.getConfig().kSettleTime)
            );
        } else {
            addCommands(
                    new InstantCommand(() -> sIntake.setDeployed(false)),
                    new WaitCommand(sIntake.getConfig().kSettleTime)
            );
        }
    }

}
