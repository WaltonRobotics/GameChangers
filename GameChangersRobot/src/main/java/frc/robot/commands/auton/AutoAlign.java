package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;

import java.util.logging.Level;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;
import static frc.robot.Robot.sDrivetrain;

public class AutoAlign extends CommandBase {

    private double targetHeading;

    public AutoAlign() {
        addRequirements(sDrivetrain);
    }

    @Override
    public void initialize() {
        targetHeading = sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX();

        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Auto align turning to: " + targetHeading);

        sDrivetrain.getTurnPID().reset(new TrapezoidProfile.State(sDrivetrain.getHeading().getDegrees(),
                sDrivetrain.getAngularVelocityDegreesPerSec()));

        SmartDashboard.putNumber(kAutoAlignHeadingSetpointKey, targetHeading);
    }

    @Override
    public void execute() {
        if (kIsInTuningMode) {
            sDrivetrain.getTurnPID().setP(SmartDashboard.getNumber(kAutoAlignTurnPKey,
                    sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getP()));

            sDrivetrain.getTurnPID().setI(SmartDashboard.getNumber(kAutoAlignTurnIKey,
                    sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getI()));

            sDrivetrain.getTurnPID().setD(SmartDashboard.getNumber(kAutoAlignTurnDKey,
                    sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getD()));
        }

        double turnRate = -sDrivetrain.getTurnPID().calculate(sDrivetrain.getHeading().getDegrees(), targetHeading);

        SmartDashboard.putNumber(kAutoAlignPositionErrorKey, sDrivetrain.getTurnPID().getPositionError());
        SmartDashboard.putNumber(kAutoAlignVelocityErrorKey, sDrivetrain.getTurnPID().getVelocityError());
        SmartDashboard.putNumber(kAutoAlignTurnRateKey, turnRate);

        sDrivetrain.setDutyCycles(turnRate, -turnRate);
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return sDrivetrain.getTurnPID().atSetpoint();
    }
}
