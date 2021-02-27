package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sConveyor;

public class ConveyorCommand extends CommandBase {
    public ConveyorCommand() {
        addRequirements(sConveyor);
    }


}
