package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;
import frc.robot.subsystems.ProMicro;
import frc.robot.vision.PixyCamHelper;

import java.util.Map;

import static frc.robot.Constants.Field.kGalacticSearchBreakPlaneLineMeters;
import static frc.robot.Paths.AutonavPaths.BouncePaths.*;
import static frc.robot.Paths.AutonavPaths.sBarrelRacingTrajectory;
import static frc.robot.Paths.AutonavPaths.sSlalomTrajectory;
import static frc.robot.Paths.GalacticSearchPaths.*;
import static frc.robot.Robot.*;

public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),

    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),

    GALACTIC_SEARCH("Galactic Search Routine",
            new SelectCommand(
                    Map.ofEntries(
                            Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A,
                                    new SequentialCommandGroup(
                                            new SetIntakeToggle(true),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                                            new InstantCommand(() -> sDrivetrain.reset()),
                                            new ResetPose(sRedATrajectory),
                                            new RamseteTrackingCommand(sRedATrajectory, true, false),
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
                                            new ResetPose(sRedBTrajectory),
                                            new RamseteTrackingCommand(sRedBTrajectory, true, false),
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
                                            new ResetPose(sBlueATrajectory),
                                            new RamseteTrackingCommand(sBlueATrajectory, true, false),
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
                                            new ResetPose(sBlueBTrajectory),
                                            new RamseteTrackingCommand(sBlueBTrajectory, true, false),
                                            new InstantCommand(() ->
                                                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                                            new RunCommand(() -> sDrivetrain.setDutyCycles(1.0, 1.0))
                                                    .withInterrupt(() -> sDrivetrain.getCurrentPose().getX()
                                                            >= kGalacticSearchBreakPlaneLineMeters),
                                            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0))
                                    )
                            )
                    ),
                    () -> ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A
                    //PixyCamHelper::getGalacticSearchDetermination
            )
    ),

    BARREL_RACING("Barrel Racing Path",
            new SequentialCommandGroup(
                    new ResetPose(sBlueBTrajectory),
                    new RamseteTrackingCommand(sBarrelRacingTrajectory, true, false)
            )
    ),

    SLALOM("Slalom Path",
            new SequentialCommandGroup(
                    new ResetPose(sSlalomTrajectory),
                    new RamseteTrackingCommand(sSlalomTrajectory, true, false)
            )
    ),

    BOUNCE("Bounce Path",
            new SequentialCommandGroup(
                    new ResetPose(sBounce1Trajectory),
                    new RamseteTrackingCommand(sBounce1Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce2Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce3Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce4Trajectory, true, false)
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
