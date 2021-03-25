package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.*;
import frc.robot.commands.auton.shootingChallenges.AutoAssistDriveStraight;
import frc.robot.commands.auton.shootingChallenges.AutoHomeForInterstellarAccuracy;
import frc.robot.commands.auton.shootingChallenges.CalibratePose;

import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.*;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
        sAutoAlignButton.whenPressed(new AutoAlign().withTimeout(kAutoAlignTimeout));
        sAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());
        sSecondaryAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());
        sTertiaryAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());

        sCalibratePoseButton.whenPressed(new CalibratePose());
        sHomeInterstellarAccuracyButton.whenPressed(new AutoHomeForInterstellarAccuracy());
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
