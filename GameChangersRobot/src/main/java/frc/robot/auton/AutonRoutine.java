package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.auton.ResetPose;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.commands.auton.shootingChallenges.AlignTurret;
import frc.robot.commands.auton.shootingChallenges.ShootAllBalls;
import frc.robot.commands.characterization.DrivetrainCharacterizationRoutine;
import frc.robot.commands.tuning.FindAngularMaxVelAccel;
import frc.robot.commands.tuning.FindLinearMaxVelAccel;
import frc.robot.commands.tuning.FindTurretAngularMaxVelAccel;

import static frc.robot.Paths.MiscellaneousTrajectories.sTestTrajectory;
import static frc.robot.Paths.RoutineFive.*;
import static frc.robot.Paths.RoutineFour.*;
import static frc.robot.Paths.RoutineOne.sBackupToShootThree;
import static frc.robot.Paths.RoutineOne.sPickupThreeFromTrench;
import static frc.robot.Paths.RoutineThree.sBackupToShootFive;
import static frc.robot.Paths.RoutineThree.sPickupFiveFromTrench;
import static frc.robot.Paths.RoutineTwo.*;
import static frc.robot.Paths.RoutineZero.sBackwards;
import static frc.robot.Paths.RoutineZero.sForwards;
import static frc.robot.Robot.sDrivetrain;


public enum AutonRoutine {

    DO_NOTHING("Do Nothing", new SequentialCommandGroup()),

    ROUTINE_ZERO_A("Cross Baseline Forwards", new SequentialCommandGroup(
            new ResetPose(sForwards),
            new RamseteTrackingCommand(sForwards, true, false)
    )),

    ROUTINE_ZERO_B("Cross Baseline Backwards", new SequentialCommandGroup(
            new ResetPose(sBackwards),
            new RamseteTrackingCommand(sBackwards, true, false)
    )),

    ROUTINE_ZERO_C("Shoot 3, Cross Baseline Forwards", new SequentialCommandGroup(
            new AlignTurret(),
            new ShootAllBalls(3, 7.5),
            new ResetPose(sForwards),
            new RamseteTrackingCommand(sForwards, true, false)
    )),

    ROUTINE_ZERO_D("Shoot 3, Cross Baseline Backwards", new SequentialCommandGroup(
            new AlignTurret(),
            new ShootAllBalls(3, 7.5),
            new ResetPose(sBackwards),
            new RamseteTrackingCommand(sBackwards, true, false)
    )),

    // 13
    // 18
    ROUTINE_ONE("Shoot 3, Pickup 3 From Our Trench, Shoot 3 (6pt)", new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SetIntakeToggle(true),
                    new SequentialCommandGroup(
                            new AlignTurret(),
                            new ShootAllBalls(3, 4)
                    )
            ),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupThreeFromTrench),
            new RamseteTrackingCommand(sPickupThreeFromTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootThree, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(3, 4)
    )),

    ROUTINE_TWO_A("Pickup 2 from Enemy Trench, Shoot 5 (6pt)", new SequentialCommandGroup(
            new SetIntakeToggle(true),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupTwoFromEnemyTrench),
            new RamseteTrackingCommand(sPickupTwoFromEnemyTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootForSix, true, false),
            new AlignTurret(),
            new ShootAllBalls(5, 4)
    )),

    ROUTINE_TWO_B("Pickup 2 from Enemy Trench, Shoot 5 (4pt)", new SequentialCommandGroup(
            new SetIntakeToggle(true),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupTwoFromEnemyTrench),
            new RamseteTrackingCommand(sPickupTwoFromEnemyTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootForFour, true, false),
            new AlignTurret(),
            new ShootAllBalls(5, 4)
    )),

    ROUTINE_THREE("Shoot 3, Pickup 5 From Our Trench, Shoot 5 (6pt)", new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SetIntakeToggle(true),
                    new SequentialCommandGroup(
                            new AlignTurret(),
                            new ShootAllBalls(3, 4)
                    )
            ),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupFiveFromTrench),
            new RamseteTrackingCommand(sPickupFiveFromTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootFive, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(5, 4)
    )),

    ROUTINE_FOUR("Shoot 3, Pickup 3 From Our Trench, Pickup 2 From Rendezvous, Shoot 5 (6pt)",
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new SetIntakeToggle(true),
                            new SequentialCommandGroup(
                                    new AlignTurret(),
                                    new ShootAllBalls(3, 4)
                            )
                    ),
                    new InstantCommand(() ->
                            AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                    new ResetPose(sPickupThreeFromTrench),
                    new RamseteTrackingCommand(sPickupThreeFromTrench, true, false),
                    new InstantCommand(() ->
                            AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                    new RamseteTrackingCommand(sBackupToAlignIntake, true, false),
                    new InstantCommand(() ->
                            AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
                    new RamseteTrackingCommand(sGoIntoRendezvousForTwo, true, false),
                    new InstantCommand(() ->
                            AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
                    new RamseteTrackingCommand(sBackupToShoot, true, false),
                    new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
                    new AlignTurret(),
                    new ShootAllBalls(5, 4)
            )
    ),

    ROUTINE_FIVE_A("Pickup 2 From Rendezvous, Shoot 5 for 6pt", new SequentialCommandGroup(
            new SetIntakeToggle(true),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupTwoInRendezvous),
            new RamseteTrackingCommand(sPickupTwoInRendezvous, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(Paths.RoutineFive.sBackupToShootFive, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(5, 4)
    )),

    ROUTINE_FIVE_B("Shoot 3, Pickup 4 From Rendezvous, Shoot 4 for 6pt", new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SetIntakeToggle(true),
                    new SequentialCommandGroup(
                            new AlignTurret(),
                            new ShootAllBalls(3, 4)
                    )
            ),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupFourInRendezvous),
            new RamseteTrackingCommand(sPickupFourInRendezvous, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootFour, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(4, 4)
    )),

    ROUTINE_SIX("Pickup 2 in Enemy Trench, Shoot 5, Pickup 2 in Rendezvous, Shoot 2", new SequentialCommandGroup(
            new SetIntakeToggle(true),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(sPickupTwoFromEnemyTrench),
            new RamseteTrackingCommand(sPickupTwoFromEnemyTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(sBackupToShootForSix, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(5, 4),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new RamseteTrackingCommand(Paths.RoutineSix.sPickupTwoInRendezvous, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(Paths.RoutineFive.sBackupToShootFive, true, false), // Actually shooting 2
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(2, 4)
    )),

    ROUTINE_SEVEN("Shoot 3, Pickup 5 From Enemy Trench, Shoot 5", new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SetIntakeToggle(true),
                    new SequentialCommandGroup(
                            new AlignTurret(),
                            new ShootAllBalls(3, 4)
                    )
            ),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(true)),
            new ResetPose(Paths.RoutineSeven.sPickupTwoInEnemyTrench),
            new RamseteTrackingCommand(Paths.RoutineSeven.sPickupTwoInEnemyTrench, true, false),
            new RamseteTrackingCommand(Paths.RoutineSeven.sPickupExtraThreeInEnemyTrench, true, false),
            new InstantCommand(() ->
                    AutonFlags.getInstance().setDoesAutonNeedToIntake(false)),
            new RamseteTrackingCommand(Paths.RoutineSeven.sBackoutFromTrench, true, false),
            new RamseteTrackingCommand(Paths.RoutineSeven.sBackupToShootFive, true, false),
            new InstantCommand(() -> sDrivetrain.setDutyCycles(0.0, 0.0)),
            new AlignTurret(),
            new ShootAllBalls(5, 4)
    )),

    DRIVETRAIN_CHARACTERIZATION("Drivetrain Characterization", new DrivetrainCharacterizationRoutine()),

    FIND_MAX_LINEAR_VEL_ACCEL("Find Maximum Drivetrain Linear Velocity and Acceleration",
            new FindLinearMaxVelAccel(10.0)),

    FIND_MAX_ANGULAR_VEL_ACCEL("Find Maximum Drivetrain Angular Velocity and Acceleration",
            new FindAngularMaxVelAccel(10.0)),

    FIND_TURRET_MAX_VEL_ACCEL("Find Maximum Turret Angular Velocity and Acceleration",
            new FindTurretAngularMaxVelAccel(3.0)),

    TEST_TURN_90("Turn CCW 90 Degrees", new SequentialCommandGroup(
            new InstantCommand(() -> sDrivetrain.reset()),
            new TurnToAngle(90)
    )),

    TEST_TRAJECTORY("Simple Spline",
            new SequentialCommandGroup(
                    new ResetPose(sTestTrajectory),
                    new RamseteTrackingCommand(sTestTrajectory, true, false)
            )
    );

    /*
    GALACTIC_SEARCH("Galactic Search Routine",
            new SequentialCommandGroup(
                    new SetIntakeToggle(true),
                    new SelectCommand(
                            Map.ofEntries(
                                    Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A,
                                            new SequentialCommandGroup(
                                                    new ParallelDeadlineGroup(
                                                            new SequentialCommandGroup(
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
                                                            ),
                                                            new IntakeCommand()
                                                    )
                                            )
                                    ),

                                    Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_B,
                                            new SequentialCommandGroup(
                                                    new ParallelDeadlineGroup(
                                                            new SequentialCommandGroup(
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
                                                            ),
                                                            new IntakeCommand()
                                                    )
                                            )
                                    ),

                                    Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_BLUE_A,
                                            new SequentialCommandGroup(
                                                    new ParallelDeadlineGroup(
                                                            new SequentialCommandGroup(
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
                                                            ),
                                                            new IntakeCommand()
                                                    )
                                            )
                                    ),

                                    Map.entry(ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_BLUE_B,
                                            new SequentialCommandGroup(
                                                    new ParallelDeadlineGroup(
                                                            new SequentialCommandGroup(
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
                                                            ),
                                                            new IntakeCommand()
                                                    )
                                            )
                                    )
                            ),
//                    () -> ProMicro.PixyCamReadMessage.GALACTIC_SEARCH_RED_A,
                            PixyCamHelper::getGalacticSearchDetermination
                    )
            )
    ),

    BARREL_RACING("Barrel Racing Path",
            new SequentialCommandGroup(
                    new SetIntakeToggle(false),
                    new ResetPose(sBarrelRacingTrajectory),
                    new RamseteTrackingCommand(sBarrelRacingTrajectory, true, false)
            )
    ),

    SLALOM("Slalom Path",
            new SequentialCommandGroup(
                    new SetIntakeToggle(false),
                    new ResetPose(sSlalomTrajectory),
                    new RamseteTrackingCommand(sSlalomTrajectory, true, false)
            )
    ),

    BOUNCE("Bounce Path",
            new SequentialCommandGroup(
                    new SetIntakeToggle(false),
                    new ResetPose(sBounce1Trajectory),
                    new RamseteTrackingCommand(sBounce1Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce2Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce3Trajectory, true, false),
                    new RamseteTrackingCommand(sBounce4Trajectory, true, false)
            )
    );
     */

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
