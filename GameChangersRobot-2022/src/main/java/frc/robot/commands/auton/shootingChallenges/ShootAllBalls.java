package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auton.AutonFlags;

import static frc.robot.Robot.sConveyor;

public class ShootAllBalls extends SequentialCommandGroup {

    public ShootAllBalls(int numberOfBalls, double timeoutSeconds) {
        SequentialCommandGroup waitForAllBallsToBeShot = new SequentialCommandGroup();

        for (int i = 0; i < numberOfBalls; i++) {
            waitForAllBallsToBeShot.addCommands(new WaitUntilCommand(()
                    -> sConveyor.getBackConveyorBool().isFallingEdge()));
        }

        addCommands(
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToShoot(true)),
                waitForAllBallsToBeShot.withTimeout(timeoutSeconds),
                new InstantCommand(() -> AutonFlags.getInstance().setDoesAutonNeedToShoot(false))
        );
    }

}
