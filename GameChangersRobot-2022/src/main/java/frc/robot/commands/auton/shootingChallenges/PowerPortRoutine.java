package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.AutonFlags;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Constants.Field.kPowerPortScoringZonePose;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sIntake;

public class PowerPortRoutine extends SequentialCommandGroup {

    public PowerPortRoutine() {
        addCommands(
                new InstantCommand(() -> sIntake.setDeployed(false)),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
                new InstantCommand(() -> sDrivetrain.resetPose(kPowerPortScoringZonePose)),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsTurretZeroingDisabled(true)),
//                new AlignTurret(),
                new ShootAllBalls(3, 5),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false)),
                new WaitCommand(0.5),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsTurretZeroingDisabled(false))
        );
    }

}
