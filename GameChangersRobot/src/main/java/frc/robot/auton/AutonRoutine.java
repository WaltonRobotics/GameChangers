package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;
import frc.robot.subsystems.ProMicro;
import frc.robot.vision.PixyCamHelper;

import java.util.function.Supplier;

import static frc.robot.Paths.GalacticSearchPaths.*;
import static frc.robot.Robot.sDrivetrain;

public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),

    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),

    GALACTIC_SEARCH("Galactic Search",
            new ConditionalCommand(
                    // Red A
                    new SequentialCommandGroup(
                            new InstantCommand(() -> sDrivetrain.reset()),
                            new ResetPose(sRedA),
                            new SetIntakeToggle(true, 0.5),
                            new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                            new RamseteTrackingCommand(sRedA, true, false),
                            new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToIntake(false))
                    ),
                    new ConditionalCommand(
                            // Red B
                            new SequentialCommandGroup(),
                            new ConditionalCommand(
                                    // Blue A
                                    new SequentialCommandGroup(),
                                    // Blue B
                                    new SequentialCommandGroup(),
                                    () -> PixyCamHelper.getGalacticSearchDetermination()
                                            == ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_BLUE_A
                            ),
                            () -> PixyCamHelper.getGalacticSearchDetermination()
                                    == ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_B
                    ),
                    () -> PixyCamHelper.getGalacticSearchDetermination()
                            == ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A
            )
    );

    private final String mDescription;
    private final CommandBase mCommandGroup;

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
