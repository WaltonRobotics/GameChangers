package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.LimelightHelper;
import frc.robot.vision.PixyCamHelper;

import static frc.robot.Robot.sProMicro;
import static frc.robot.subsystems.ProMicro.LEDStripWriteMessage.*;

public class ProMicroCommand extends CommandBase {

    public ProMicroCommand() {
        addRequirements(sProMicro);
    }

    @Override
    public void execute() {
        if (LimelightHelper.getTV() == 1) {                     // Limelight sees a target
            if (Math.abs(LimelightHelper.getTX()) <= 1) {         // Within angle tolerance
                sProMicro.setLEDStripMessage(ALIGNED);
            } else if (LimelightHelper.getTX() < 0) {             // Target is to the left
                sProMicro.setLEDStripMessage(TURN_LEFT);
            } else {                                              // Target is to the right
                sProMicro.setLEDStripMessage(TURN_RIGHT);
            }
        } else {
            sProMicro.setLEDStripMessage(IDLE);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
