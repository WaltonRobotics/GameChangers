package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UtilMethods;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sDrivetrain;

public class DriveStraight extends CommandBase {

    private final double mDesiredDistance;
    private double mInitialLeftPosition;
    private double mInitialRightPosition;
    private double mInitialHeading;

    public DriveStraight(double desiredDistance) {
        addRequirements(sDrivetrain);

        this.mDesiredDistance = desiredDistance;
    }

    @Override
    public void initialize() {
//        drivetrain.getmDriveStraightPowerController().setP(1);
//        System.out.println("P value set");
//        sDrivetrain.reset();
        mInitialLeftPosition = sDrivetrain.getLeftPositionMeters();
        mInitialRightPosition = sDrivetrain.getRightPositionMeters();
        mInitialHeading = getHeading();

        sDrivetrain.getDriveStraightPowerProfiledPID().reset(
                new TrapezoidProfile.State(0, getVelocityAverage()));

        sDrivetrain.getDriveStraightHeadingProfiledPID().reset(
                new TrapezoidProfile.State(0, sDrivetrain.getAngularVelocityDegreesPerSec()));

//        drivetrain.getmDriveStraightPowerController().setTolerance(0.09);
//        drivetrain.getmDriveStraightHeadingPIDController().setTolerance(1);
//        System.out.println("Initialize completed!");
    }

    private double getVelocityAverage() {
        return Math.abs(sDrivetrain.getLeftVelocityMetersPerSec() + sDrivetrain.getRightVelocityMetersPerSec()) / 2;
    }

    private double getDistanceAverage() {
        return Math.abs((sDrivetrain.getLeftPositionMeters() - mInitialLeftPosition)
                + (sDrivetrain.getRightPositionMeters() - mInitialRightPosition)) / 2;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber(kDriveStraightDistanceAverageKey, getDistanceAverage());

        if (kIsInTuningMode) {
            sDrivetrain.getDriveStraightPowerProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightForwardPKey, sDrivetrain.getDriveStraightPowerProfiledPID().getP()));
            sDrivetrain.getDriveStraightPowerProfiledPID().setI(SmartDashboard.getNumber(
                    kDriveStraightForwardIKey, sDrivetrain.getDriveStraightPowerProfiledPID().getI()));
            sDrivetrain.getDriveStraightPowerProfiledPID().setD(SmartDashboard.getNumber(
                    kDriveStraightForwardDKey, sDrivetrain.getDriveStraightPowerProfiledPID().getD()));

            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingPKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getP()));
            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingIKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getI()));
            sDrivetrain.getDriveStraightHeadingProfiledPID().setP(SmartDashboard.getNumber(
                    kDriveStraightHeadingDKey, sDrivetrain.getDriveStraightHeadingProfiledPID().getD()));
        }

        double forward = sDrivetrain.getDriveStraightPowerProfiledPID().calculate(getDistanceAverage(),
                mDesiredDistance);

        double turnRate = -Math.signum(forward) * sDrivetrain.getDriveStraightHeadingProfiledPID().calculate(
                getHeading() - mInitialHeading, 0);

        SmartDashboard.putNumber(kDriveStraightForwardErrorKey,
                sDrivetrain.getDriveStraightPowerProfiledPID().getPositionError());
        SmartDashboard.putNumber(kDriveStraightForwardRateKey, forward);


        SmartDashboard.putNumber(kDriveStraightHeadingErrorKey,
                sDrivetrain.getDriveStraightHeadingProfiledPID().getPositionError());
        SmartDashboard.putNumber(kDriveStraightTurnRateKey, turnRate);

        sDrivetrain.setArcadeSpeeds(forward, turnRate);
    }

    @Override
    public boolean isFinished() {
        return sDrivetrain.getDriveStraightPowerProfiledPID().atSetpoint()
                && sDrivetrain.getDriveStraightHeadingProfiledPID().atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    private double getHeading() {
        return UtilMethods.restrictAngle(sDrivetrain.getHeading().getDegrees(), -180.0, 180.0);
    }
}
