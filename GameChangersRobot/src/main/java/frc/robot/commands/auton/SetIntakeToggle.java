package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Robot.sIntake;

public class SetIntakeToggle extends SequentialCommandGroup {

    public SetIntakeToggle(boolean state) {
        addCommands(
                new InstantCommand(() -> sIntake.setDeployed(state)),
                new WaitCommand(sIntake.getConfig().kSettleTime)
        );
    }

}
