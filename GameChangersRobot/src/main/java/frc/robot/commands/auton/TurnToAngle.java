package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DebuggingLog;

import java.util.function.DoubleSupplier;
import java.util.logging.Level;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sDrivetrain;

public class TurnToAngle extends CommandBase {

    private final DoubleSupplier mTargetHeadingSupplier;
    private double mTargetHeading;

    public TurnToAngle(DoubleSupplier targetHeadingSupplier) {
        addRequirements(sDrivetrain);

        this.mTargetHeadingSupplier = targetHeadingSupplier;
    }

    public TurnToAngle(double targetHeading) {
        this.mTargetHeadingSupplier = () -> targetHeading;
    }

    @Override
    public void initialize() {
        mTargetHeading = mTargetHeadingSupplier.getAsDouble();

        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Turning to heading: "
                + mTargetHeading);

        sDrivetrain.getTurnProfiledPID().reset(new TrapezoidProfile.State(sDrivetrain.getHeading().getDegrees(),
                sDrivetrain.getAngularVelocityDegreesPerSec()));

        SmartDashboard.putNumber(kTurnToAngleHeadingSetpointKey, mTargetHeading);
    }

    @Override
    public void execute() {
        if (kIsInTuningMode) {
            sDrivetrain.getTurnProfiledPID().setP(SmartDashboard.getNumber(kTurnToAnglePKey,
                    sDrivetrain.getTurnProfiledPID().getP()));

            sDrivetrain.getTurnProfiledPID().setI(SmartDashboard.getNumber(kTurnToAngleIKey,
                    sDrivetrain.getTurnProfiledPID().getI()));

            sDrivetrain.getTurnProfiledPID().setD(SmartDashboard.getNumber(kTurnToAngleDKey,
                    sDrivetrain.getTurnProfiledPID().getD()));
        }

        double turnRate = -sDrivetrain.getTurnProfiledPID().calculate(sDrivetrain.getHeading().getDegrees(),
                mTargetHeading);

        SmartDashboard.putNumber(kTurnToAnglePositionErrorKey, sDrivetrain.getTurnProfiledPID().getPositionError());
        SmartDashboard.putNumber(kTurnToAngleVelocityErrorKey, sDrivetrain.getTurnProfiledPID().getVelocityError());
        SmartDashboard.putNumber(kTurnToAngleRateKey, turnRate);

        sDrivetrain.setDutyCycles(turnRate, -turnRate);
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return sDrivetrain.getTurnProfiledPID().atSetpoint();
    }

}
