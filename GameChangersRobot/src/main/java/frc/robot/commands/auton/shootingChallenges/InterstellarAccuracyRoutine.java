package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auton.AutonFlags;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Robot.sDrivetrain;

public class InterstellarAccuracyRoutine extends SequentialCommandGroup {

    public InterstellarAccuracyRoutine() {
        addCommands(
            new InstantCommand(() -> sDrivetrain.setHeading(90.0)),
            new ParallelCommandGroup(
                    new WaitUntilCommand(() -> SubsystemFlags.getInstance().hasTurretZeroed()),
                    new SetIntakeToggle(false)
            ),
            new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
            new AlignTurret(),
            new ShootAllBalls(3, 10.0),
            new TurnToAngle(180.0).withTimeout(2.5),
            new AlignTurret(),
            new WaitCommand(0.5),
//            new InstantCommand(() -> sIntake.setDeployed(true)),
            new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false))
        );
    }

}
