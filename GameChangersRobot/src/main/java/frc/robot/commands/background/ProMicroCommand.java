package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sProMicro;
import static frc.robot.subsystems.ProMicro.LEDStripWriteLineState.ALIGNED_AND_IN_RANGE;

public class ProMicroCommand extends CommandBase {

    public ProMicroCommand() {
        addRequirements(sProMicro);
    }

    @Override
    public void execute() {
        sProMicro.setLEDStripState(ALIGNED_AND_IN_RANGE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
