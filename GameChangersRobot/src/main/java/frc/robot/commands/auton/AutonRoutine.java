package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;

public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),
    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),
    GALACTIC_SEARCH_RED_A("Galactic Search Red A", new SequentialCommandGroup()),
    GALACTIC_SEARCH_BLUE_A("Galactic Search Blue A", new SequentialCommandGroup()),
    GALACTIC_SEARCH_RED_B("Galactic Search Red B", new SequentialCommandGroup()),
    GALACTIC_SEARCH_BLUE_B("Galactic Search Blue B", new SequentialCommandGroup());

    String name;
    CommandBase commandGroup;

    AutonRoutine(String name, CommandBase commandGroup) {
        this.name = name;
        this.commandGroup = commandGroup;
    }

    public String getName() {
        return name;
    }

    public CommandBase getCommandGroup() {
        return commandGroup;
    }

    @Override
    public String toString() {
        return name;
    }
}
