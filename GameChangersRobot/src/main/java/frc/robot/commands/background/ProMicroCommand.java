package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.LimelightConstants.kAlignedToleranceDegrees;
import static frc.robot.Robot.sProMicro;

public class ProMicroCommand extends CommandBase {

    public ProMicroCommand() {
        addRequirements(sProMicro);
    }

    @Override
    public void execute() {
        if (LimelightHelper.getTV() == 1) {
            if (UtilMethods.isWithinTolerance(LimelightHelper.getTX(), 0, kAlignedToleranceDegrees)) {         // Within angle tolerance
                sProMicro.setLEDAlignedMode();
            } else if (LimelightHelper.getTX() < 0) {
                sProMicro.setLEDTurnLeftMode();
            } else {
                sProMicro.setLEDTurnRightMode();
            }
        } else {
            sProMicro.setLEDIdleMode();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
