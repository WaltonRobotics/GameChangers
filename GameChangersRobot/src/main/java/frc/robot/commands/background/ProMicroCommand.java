package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sProMicro;
import static frc.robot.subsystems.ProMicro.LEDStripWriteMessage.IDLE;

public class ProMicroCommand extends CommandBase {

    public ProMicroCommand() {
        addRequirements(sProMicro);
    }

    @Override
    public void execute() {
        sProMicro.setLEDStripMessage(IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
