package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auton.AutoAlign;
import frc.robot.commands.auton.shootingChallenges.AutoAssistDriveStraight;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.ContextFlags.kIsInfiniteRecharge;
import static frc.robot.Constants.DriverPreferences.kAutoAlignTimeout;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDriveModeChooser;
import static frc.robot.Robot.sDrivetrain;

public class DriveCommand extends CommandBase {

    public DriveCommand() {
        addRequirements(sDrivetrain);

        sResetDrivetrainButton.whenPressed(() -> sDrivetrain.reset());
        sAutoAlignButton.whenPressed(new AutoAlign().withTimeout(kAutoAlignTimeout));

        if (!kIsInfiniteRecharge) {
            sAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());
            sSecondaryAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());
            sTertiaryAutoAssistDriveStraightButton.whenPressed(new AutoAssistDriveStraight());

//            sCalibratePoseButton.whenPressed(new CalibratePose());
//            sHomeInterstellarAccuracyButton.whenPressed(new AutoHomeForInterstellarAccuracy());
//            sHomePowerPortButton.whenPressed(new GoToScoringZone());
//            sGoToReintroductionZoneButton.whenPressed(new GoToReintroductionZone());
        }

        if (kIsInTuningMode) {
//            sBackUpTwoFeetForShooterCalibrationButton.whenPressed(new BackUpTwoFeetForShooterCalibration());
        }
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