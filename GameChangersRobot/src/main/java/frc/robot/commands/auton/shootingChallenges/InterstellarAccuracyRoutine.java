package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auton.AutonFlags;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sIntake;

public class InterstellarAccuracyRoutine extends SequentialCommandGroup {

    public InterstellarAccuracyRoutine() {
        addCommands(
            new InstantCommand(() -> sDrivetrain.setHeading(90.0)),
            new WaitUntilCommand(() -> SubsystemFlags.getInstance().hasTurretZeroed()),
            new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
            new AlignTurret(),
            new ShootAllBalls(3, 10.0),
            new TurnToAngle(180.0).withTimeout(2.0),
            new AlignTurret(),
            new WaitCommand(0.5),
//            new InstantCommand(() -> sIntake.setDeployed(true)),
            new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false))
        );
    }

}
