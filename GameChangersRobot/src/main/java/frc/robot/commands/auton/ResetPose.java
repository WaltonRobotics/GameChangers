package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Paths;


import static frc.robot.Robot.sDrivetrain;

public class ResetPose extends InstantCommand {

    private Trajectory trajectory;

    public ResetPose(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        sDrivetrain.resetPose(trajectory.getInitialPose());
    }
}
