package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auton.AutonFlags;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Constants.Field.kPowerPortScoringZonePose;
import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sIntake;

public class PowerPortRoutine extends SequentialCommandGroup {

    public PowerPortRoutine() {
        addCommands(
                new InstantCommand(() -> sIntake.setDeployed(false)),
                new InstantCommand(() -> sIntake.setRetracted(true)),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(true)),
                new InstantCommand(() -> sDrivetrain.resetPose(kPowerPortScoringZonePose)),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsZeroingDisabled(true)),
//                new AlignTurret(),
                new ShootAllBalls(3, 5),
                new InstantCommand(() -> AutonFlags.getInstance().setIsInAuton(false)),
                new WaitCommand(0.5),
                new InstantCommand(() -> SubsystemFlags.getInstance().setIsZeroingDisabled(false))
        );
    }

}