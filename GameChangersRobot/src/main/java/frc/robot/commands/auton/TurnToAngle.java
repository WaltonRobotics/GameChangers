package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sDrivetrain;

public class TurnToAngle extends CommandBase {

    private double targetAngle;

    public TurnToAngle(double targetAngle) {
        addRequirements(sDrivetrain);
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        System.out.println("turning to " + targetAngle);

        sDrivetrain.getTurnPID().reset(new TrapezoidProfile.State(sDrivetrain.getHeading().getDegrees(), 0));

        SmartDashboard.putNumber("Turn Setpoint", targetAngle);
    }

    @Override
    public void execute() {
        sDrivetrain.getTurnPID().setP(SmartDashboard.getNumber("Turn P", 0.05));
        double turnRate = -sDrivetrain.getTurnPID().calculate(sDrivetrain.getHeading().getDegrees(), targetAngle);
        SmartDashboard.putNumber("Velocity error", sDrivetrain.getTurnPID().getVelocityError());
        SmartDashboard.putNumber("Position error", sDrivetrain.getTurnPID().getPositionError());
        System.out.println(sDrivetrain.getTurnPID().getPositionError());
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
