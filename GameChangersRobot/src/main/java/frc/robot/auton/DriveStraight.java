package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;

public class DriveStraight extends CommandBase {

    private double desiredDistance;

    public DriveStraight(double desiredDistance) {
        addRequirements(drivetrain);

        this.desiredDistance = desiredDistance;
    }

    @Override
    public void initialize() {
        drivetrain.reset();
        drivetrain.getmDriveStraightHeadingPIDController().reset(new TrapezoidProfile.State(0, 0));
    }

    private double distanceAverage() {
        return (drivetrain.leftMetersTravelled() + drivetrain.rightMetersTravelled()) / 2;
    }

    @Override
    public void execute() {
        drivetrain.getmDriveStraightHeadingPIDController().setP(SmartDashboard.getNumber("Drive Straight Heading P", 0.19));
        drivetrain.getmDriveStraightPowerController().setP(SmartDashboard.getNumber("Forward P", 0.1));
        double turnRate = -drivetrain.getmDriveStraightHeadingPIDController().calculate(drivetrain.getHeading().getDegrees(), 0);
        double forward = drivetrain.getmDriveStraightPowerController().calculate(distanceAverage(), desiredDistance);

        SmartDashboard.putNumber("Turn error", drivetrain.getmDriveStraightHeadingPIDController().getPositionError());

        SmartDashboard.putNumber("Turn rate", turnRate);
        SmartDashboard.putNumber("Forward rate", forward);
        drivetrain.setArcadeSpeeds(forward, turnRate);
    }

    @Override
    public boolean isFinished() {
        return Math.abs((drivetrain.leftMetersTravelled() + drivetrain.rightMetersTravelled()) / 2) >= desiredDistance;
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDutyCycles(0, 0);
    }
}
