package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.AutoAlign;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.*;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        if (kIsInTuningMode) {
            sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
        }

        sAutoAlignButton.whenPressed(new AutoAlign().withTimeout(kAutoAlignTimeout));
    }

    @Override
    public void execute() {
        sDriveModeChooser.getSelected().feed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
