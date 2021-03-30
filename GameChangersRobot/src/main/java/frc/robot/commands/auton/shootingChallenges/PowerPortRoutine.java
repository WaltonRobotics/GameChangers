package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auton.AutonFlags;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Robot.sDrivetrain;

public class PowerPortRoutine extends SequentialCommandGroup {

    public PowerPortRoutine() {
        addCommands(
                new WaitUntilCommand(() -> SubsystemFlags.getInstance().hasTurretZeroed()),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
                new InstantCommand(() -> sDrivetrain.setHeading(180.0)),
                new AlignTurret(),
                new ShootAllBalls(3, 10.0),
                new TurnToAngle(180.0).withTimeout(2.0),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false))
        );
    }

}
