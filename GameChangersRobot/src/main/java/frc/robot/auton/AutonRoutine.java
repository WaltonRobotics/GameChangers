package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;

import static frc.robot.Paths.GalacticSearchPaths.*;
import static frc.robot.Robot.sDrivetrain;

public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),

    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),

    GALACTIC_SEARCH_RED_A("Galactic Search Red A", new SequentialCommandGroup(
            new InstantCommand(() -> sDrivetrain.reset()),
            new ResetPose(sRedA),
            new SetIntakeToggle(true, 0.5),
            new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new RamseteTrackingCommand(sRedA, true, false),
            new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToIntake(false))
    )),

    GALACTIC_SEARCH_BLUE_A("Galactic Search Blue A", new SequentialCommandGroup(
            new ResetPose(sBlueA),
            new RamseteTrackingCommand(sBlueA, true, false))
    ),

    GALACTIC_SEARCH_RED_B("Galactic Search Red B", new SequentialCommandGroup(
            new ResetPose(sRedB),
            new RamseteTrackingCommand(sRedB, true, false))
    ),

    GALACTIC_SEARCH_BLUE_B("Galactic Search Blue B", new SequentialCommandGroup(
            new ResetPose(sBlueB),
            new RamseteTrackingCommand(sBlueB, true, false))
    );

    String mDescription;
    CommandBase mCommandGroup;

    AutonRoutine(String description, CommandBase commandGroup) {
        this.mDescription = description;
        this.mCommandGroup = commandGroup;
    }

    public String getDescription() {
        return mDescription;
    }

    public CommandBase getCommandGroup() {
        return mCommandGroup;
    }

    @Override
    public String toString() {
        return name() + ": " + mDescription;
    }
}
