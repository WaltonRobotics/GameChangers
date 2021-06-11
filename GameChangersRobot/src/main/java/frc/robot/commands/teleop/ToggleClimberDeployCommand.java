package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFlags;

import static frc.robot.Robot.sClimber;

public class ToggleClimberDeployCommand extends CommandBase {

    public ToggleClimberDeployCommand() {
        addRequirements(sClimber);
    }

    @Override
    public void initialize() {
        SubsystemFlags.getInstance().setDoesTurretNeedToHomeForClimbing(true);
    }

    @Override
    public void end(boolean interrupted) {
        SubsystemFlags.getInstance().setDoesTurretNeedToHomeForClimbing(false);

        sClimber.toggleClimberDeployed();
    }

    @Override
    public boolean isFinished() {
        return SubsystemFlags.getInstance().isTurretHomedForClimbing();
    }

}
