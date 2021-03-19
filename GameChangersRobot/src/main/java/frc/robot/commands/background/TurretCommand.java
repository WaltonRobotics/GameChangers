package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sTurret;

public class TurretCommand extends CommandBase {

    public TurretCommand() {
        addRequirements(sTurret);
    }

}
