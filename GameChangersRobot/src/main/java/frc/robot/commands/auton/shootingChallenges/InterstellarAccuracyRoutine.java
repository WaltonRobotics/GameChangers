package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.AutonFlags;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sIntake;

public class InterstellarAccuracyRoutine extends SequentialCommandGroup {

    public InterstellarAccuracyRoutine() {
        addCommands(
                new InstantCommand(() -> sIntake.setDeployed(false)),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsZeroingDisabled(true)),
                new InstantCommand(() -> sDrivetrain.setHeading(90.0)),
                new AlignTurret(),
                new ShootAllBalls(3, 5.0),
//            new TurnToAngle(180.0).withTimeout(2.5),
                new AlignTurret(),
//            new InstantCommand(() -> sIntake.setDeployed(true)),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false)),
                new WaitCommand(0.5),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsZeroingDisabled(false))
        );
    }

}
