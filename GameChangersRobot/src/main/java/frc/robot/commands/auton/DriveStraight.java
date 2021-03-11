package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sDrivetrain;

public class DriveStraight extends CommandBase {

    private final double desiredDistance;

    public DriveStraight(double desiredDistance) {
        addRequirements(sDrivetrain);

        this.desiredDistance = desiredDistance;
    }

    @Override
    public void initialize() {
//        drivetrain.getmDriveStraightPowerController().setP(1);
        //System.out.println("P value set");
        sDrivetrain.reset();
        sDrivetrain.getDriveStraightPowerPID().reset(new TrapezoidProfile.State(desiredDistance, 0));
        sDrivetrain.getDriveStraightHeadingPID().reset(new TrapezoidProfile.State(0, 0));
//        drivetrain.getmDriveStraightPowerController().setTolerance(0.09);
//        drivetrain.getmDriveStraightHeadingPIDController().setTolerance(1);
        System.out.println("Initialize completed!");
    }

    private double getDistanceAverage() {
        return Math.abs(sDrivetrain.getLeftPositionMeters() + sDrivetrain.getRightPositionMeters()) / 2;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Distance Average", getDistanceAverage());

        sDrivetrain.getDriveStraightPowerPID().setP(SmartDashboard.getNumber("Forward P", 0.8));
        sDrivetrain.getDriveStraightHeadingPID().setP(SmartDashboard.getNumber("Drive Straight Heading P", 0.19));

        double forward = sDrivetrain.getDriveStraightPowerPID().calculate(getDistanceAverage(), desiredDistance);
        double turnRate = -sDrivetrain.getDriveStraightHeadingPID().calculate(sDrivetrain.getHeading().getDegrees(), 0);

        SmartDashboard.putNumber("Forward rate", forward);

        SmartDashboard.putNumber("Turn error", sDrivetrain.getDriveStraightHeadingPID().getPositionError());
        SmartDashboard.putNumber("Turn rate", turnRate);

        sDrivetrain.setArcadeSpeeds(forward, turnRate);
    }

    @Override
    public boolean isFinished() {
        return sDrivetrain.getDriveStraightPowerPID().atSetpoint()
                && sDrivetrain.getDriveStraightHeadingPID().atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }
}
