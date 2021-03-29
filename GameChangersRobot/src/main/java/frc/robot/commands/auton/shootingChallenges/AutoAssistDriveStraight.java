package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UtilMethods;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.kAutoAssistDriveStraightThrottleLimit;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.*;
import static frc.robot.Robot.sDriveModeChooser;
import static frc.robot.Robot.sDrivetrain;

public class AutoAssistDriveStraight extends CommandBase {

    private double mInitialHeading;

    public AutoAssistDriveStraight() {
        addRequirements(sDrivetrain);
    }

    @Override
    public void initialize() {
//        drivetrain.getmDriveStraightPowerController().setP(1);
//        System.out.println("P value set");
//        sDrivetrain.reset();
        mInitialHeading = getHeading();

        sDrivetrain.getDriveStraightHeadingProfiledPID().reset(
                new TrapezoidProfile.State(
                        mInitialHeading,
                        sDrivetrain.getAngularVelocityDegreesPerSec()
                )
        );

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

        double throttle = getThrottleAverage();

        double turnRate = -Math.signum(throttle) * sDrivetrain.getDriveStraightHeadingProfiledPID().calculate(
                getHeading(),
                mInitialHeading
        );

        SmartDashboard.putNumber(kDriveStraightHeadingErrorKey,
                sDrivetrain.getDriveStraightHeadingProfiledPID().getPositionError());
        SmartDashboard.putNumber(kDriveStraightTurnRateKey, turnRate);

        sDrivetrain.setArcadeSpeeds(throttle, turnRate);
    }

    @Override
    public boolean isFinished() {
        return !sAutoAssistDriveStraightButton.get() && !sSecondaryAutoAssistDriveStraightButton.get()
                && !sTertiaryAutoAssistDriveStraightButton.get();
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    private double getThrottleAverage() {
        return UtilMethods.limitMagnitude(sDriveModeChooser.getSelected().getLeftJoystickY()
                + sDriveModeChooser.getSelected().getRightJoystickY() / 2, kAutoAssistDriveStraightThrottleLimit);
    }

    private double getHeading() {
        return UtilMethods.restrictAngle(sDrivetrain.getHeading().getDegrees(), -180.0, 180.0);
    }
}
