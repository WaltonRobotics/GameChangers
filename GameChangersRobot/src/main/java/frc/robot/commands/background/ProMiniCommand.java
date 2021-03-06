package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sProMini;

public class ProMiniCommand extends CommandBase {

    public ProMiniCommand() {
        addRequirements(sProMini);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
