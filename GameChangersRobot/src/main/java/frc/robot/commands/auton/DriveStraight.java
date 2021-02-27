package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;

public class DriveStraight extends CommandBase {

    private double desiredDistance;

    public DriveStraight(double desiredDistance) {
        addRequirements(drivetrain);

        this.desiredDistance = desiredDistance;
        drivetrain.getmDriveStraightPowerController().setTolerance(0.09);
        drivetrain.getmDriveStraightHeadingPIDController().setTolerance(1);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
//        drivetrain.getmDriveStraightPowerController().setP(1);
        //System.out.println("P value set");
        drivetrain.reset();
        drivetrain.getmDriveStraightHeadingPIDController().reset(new TrapezoidProfile.State(desiredDistance, 0));
//        drivetrain.getmDriveStraightPowerController().setTolerance(0.09);
//        drivetrain.getmDriveStraightHeadingPIDController().setTolerance(1);
        System.out.println("Initialize completed!");
    }

    private double distanceAverage() {
        return Math.abs(drivetrain.leftMetersTravelled() + drivetrain.rightMetersTravelled()) / 2;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Distance Average", distanceAverage());
        drivetrain.getmDriveStraightHeadingPIDController().setP(SmartDashboard.getNumber("Drive Straight Heading P", 0.19));
        drivetrain.getmDriveStraightPowerController().setP(SmartDashboard.getNumber("Forward P", 0.8));
        double turnRate = -drivetrain.getmDriveStraightHeadingPIDController().calculate(drivetrain.getHeading().getDegrees(), 0);
        double forward = drivetrain.getmDriveStraightPowerController().calculate(distanceAverage(), desiredDistance);

        SmartDashboard.putNumber("Turn error", drivetrain.getmDriveStraightHeadingPIDController().getPositionError());

        SmartDashboard.putNumber("Turn rate", turnRate);
        SmartDashboard.putNumber("Forward rate", forward);
        //drivetrain.setDutyCycles(forward, forward);
        drivetrain.setArcadeSpeeds(forward, turnRate);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getmDriveStraightPowerController().atSetpoint() && drivetrain.getmDriveStraightHeadingPIDController().atSetpoint();
        //return Math.abs((drivetrain.leftMetersTravelled() + drivetrain.rightMetersTravelled()) / 2) >= desiredDistance;
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDutyCycles(0, 0);
    }
}
