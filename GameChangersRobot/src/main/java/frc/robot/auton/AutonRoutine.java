package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;
import frc.robot.subsystems.ProMicro;
import frc.robot.vision.PixyCamHelper;

import java.util.Map;
import java.util.function.Supplier;

import static frc.robot.Constants.Field.kGalacticSearchBreakPlaneLineMeters;
import static frc.robot.Paths.GalacticSearchPaths.*;
import static frc.robot.Robot.*;

public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),

    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),

    GALACTIC_SEARCH("Galactic Search",
            new SelectCommand(
                    Map.ofEntries(
                            Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A,
                                    new SequentialCommandGroup(
                                            new SetIntakeToggle(true),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                                            new InstantCommand(() -> sDrivetrain.reset()),
                                            new ResetPose(sRedA),
                                            new RamseteTrackingCommand(sRedA, true, false),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                                            new RunCommand(() -> sDrivetrain.setDutyCycles(1.0, 1.0))
                                                    .withInterrupt(() -> sDrivetrain.getCurrentPose().getX()
                                                            >= kGalacticSearchBreakPlaneLineMeters),
                                            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0))
                                    )
                            ),

                            Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_B,
                                    new SequentialCommandGroup(
                                            new SetIntakeToggle(true),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                                            new InstantCommand(() -> sDrivetrain.reset()),
                                            new ResetPose(sRedB),
                                            new RamseteTrackingCommand(sRedB, true, false),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                                            new RunCommand(() -> sDrivetrain.setDutyCycles(1.0, 1.0))
                                                    .withInterrupt(() -> sDrivetrain.getCurrentPose().getX()
                                                            >= kGalacticSearchBreakPlaneLineMeters),
                                            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0))
                                    )
                            ),

                            Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_BLUE_A,
                                    new SequentialCommandGroup(
                                            new SetIntakeToggle(true),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                                            new InstantCommand(() -> sDrivetrain.reset()),
                                            new ResetPose(sBlueA),
                                            new RamseteTrackingCommand(sBlueA, true, false),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                                            new RunCommand(() -> sDrivetrain.setDutyCycles(1.0, 1.0))
                                                    .withInterrupt(() -> sDrivetrain.getCurrentPose().getX()
                                                            >= kGalacticSearchBreakPlaneLineMeters),
                                            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0))
                                    )
                            ),

                            Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_BLUE_B,
                                    new SequentialCommandGroup(
                                            new SetIntakeToggle(true),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                                            new InstantCommand(() -> sDrivetrain.reset()),
                                            new ResetPose(sBlueB),
                                            new RamseteTrackingCommand(sBlueB, true, false),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                                            new RunCommand(() -> sDrivetrain.setDutyCycles(1.0, 1.0))
                                                    .withInterrupt(() -> sDrivetrain.getCurrentPose().getX()
                                                            >= kGalacticSearchBreakPlaneLineMeters),
                                            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0))
                                    )
                            )
                    ),
                    PixyCamHelper::getGalacticSearchDetermination
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
