package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.robots.RobotIdentifier;

import static frc.robot.Robot.sCurrentRobot;
import static frc.robot.Robot.sIntake;

public class SetIntakeToggle extends SequentialCommandGroup {

    public SetIntakeToggle(boolean state, double waitSeconds) {

        if(sCurrentRobot == RobotIdentifier.PRACTICE_GAME_CHANGERS || sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS) {
            addRequirements(sIntake);

            addCommands(new InstantCommand(() -> sIntake.setDeployed(state)));
            addCommands(new WaitCommand(waitSeconds));
        }

    }

}
