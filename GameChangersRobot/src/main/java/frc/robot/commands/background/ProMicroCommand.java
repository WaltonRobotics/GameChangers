package frc.robot.commands.background;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.SmartDashboardKeys.kTurretIsAlignedKey;
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
                SmartDashboard.putBoolean(kTurretIsAlignedKey, true);
            } else if (LimelightHelper.getTX() < 0) {             // Target is to the left
                sProMicro.setLEDStripMessage(TURN_RIGHT);
                SmartDashboard.putBoolean(kTurretIsAlignedKey, false);
            } else {                                              // Target is to the right
                sProMicro.setLEDStripMessage(TURN_LEFT);
                SmartDashboard.putBoolean(kTurretIsAlignedKey, false);
            }
        } else {
            sProMicro.setLEDStripMessage(IDLE);
            SmartDashboard.putBoolean(kTurretIsAlignedKey, false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
