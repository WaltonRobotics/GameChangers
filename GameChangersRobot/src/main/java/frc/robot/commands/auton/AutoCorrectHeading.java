package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UtilMethods;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.sAutoCorrectHeadingButton;
import static frc.robot.Robot.sDriveModeChooser;
import static frc.robot.Robot.sDrivetrain;

public class AutoCorrectHeading extends CommandBase {

    public AutoCorrectHeading() {
        addRequirements(sDrivetrain);
    }

    @Override
    public void initialize() {
//        drivetrain.getmDriveStraightPowerController().setP(1);
//        System.out.println("P value set");
//        sDrivetrain.reset();
        sDrivetrain.getDriveStraightHeadingProfiledPID().reset(
                new TrapezoidProfile.State(0, sDrivetrain.getAngularVelocityDegreesPerSec()));

//        drivetrain.getmDriveStraightPowerController().setTolerance(0.09);
//        drivetrain.getmDriveStraightHeadingPIDController().setTolerance(1);
//        System.out.println("Initialize completed!");
    }

    @Override
    public void execute() {
        if (kIsInTuningMode) {
            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingPKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getP()));
            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingIKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getI()));
            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingDKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getD()));
        }

        double turnRate = (getThrottleAverage() < 0 ? 1 : -1) * sDrivetrain.getDriveStraightHeadingProfiledPID().calculate(
                UtilMethods.restrictAngle(sDrivetrain.getHeading().getDegrees(), 0, 360), 180.0);

        SmartDashboard.putNumber(kDriveStraightHeadingErrorKey,
                sDrivetrain.getDriveStraightHeadingProfiledPID().getPositionError());
        SmartDashboard.putNumber(kDriveStraightTurnRateKey, turnRate);

        sDrivetrain.setArcadeSpeeds(getThrottleAverage(), turnRate);
    }

    @Override
    public boolean isFinished() {
        return !sAutoCorrectHeadingButton.get();
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    private double getThrottleAverage() {
        return (sDriveModeChooser.getSelected().getLeftJoystickY()
                + sDriveModeChooser.getSelected().getRightJoystickY()) / 2;
    }

}
