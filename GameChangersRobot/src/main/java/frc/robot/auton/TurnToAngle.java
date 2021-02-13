package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;

public class TurnToAngle extends CommandBase {

    private double targetAngle;

    public TurnToAngle(double targetAngle) {
        addRequirements(drivetrain);
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        System.out.println("turning to " + targetAngle);

        drivetrain.getTurnPIDController().reset(new TrapezoidProfile.State(drivetrain.getHeading().getDegrees(), 0));
        drivetrain.getTurnPIDController().setTolerance(6.0);

        SmartDashboard.putNumber("Turn Setpoint", targetAngle);
    }

    @Override
    public void execute() {
        drivetrain.getTurnPIDController().setP(SmartDashboard.getNumber("Turn P", 0.05));
        double turnRate = -drivetrain.getTurnPIDController().calculate(drivetrain.getHeading().getDegrees(), targetAngle);
        SmartDashboard.putNumber("Velocity error", drivetrain.getTurnPIDController().getVelocityError());
        SmartDashboard.putNumber("Position error", drivetrain.getTurnPIDController().getPositionError());
        System.out.println(drivetrain.getTurnPIDController().getPositionError());
        drivetrain.setDutyCycles(turnRate, -turnRate);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getTurnPIDController().atGoal();
    }
}
