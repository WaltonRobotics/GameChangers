package frc.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;
import static frc.robot.subsystems.Drivetrain.leftWheelMaster;
import static frc.robot.subsystems.Drivetrain.rightWheelMaster;

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
        leftWheelMaster.set(drivetrain.getmDriveStraightPowerController().calculate(leftWheelMaster.getEncoder().getPosition(), new TrapezoidProfile.State(desiredDistance, 0)));
        rightWheelMaster.set(drivetrain.getmDriveStraightPowerController().calculate(rightWheelMaster.getEncoder().getPosition(), new TrapezoidProfile.State(desiredDistance, 0)));
    }

    @Override
    public void execute() {
        drivetrain.getmDriveStraightHeadingPIDController().setP(SmartDashboard.getNumber("Drive Straight Heading P", 0.19));
        double turnRate = -drivetrain.getmDriveStraightHeadingPIDController().calculate(drivetrain.getHeading().getDegrees(), 0);

        SmartDashboard.putNumber("Error", drivetrain.getmDriveStraightHeadingPIDController().getPositionError());

        SmartDashboard.putNumber("Turn rate", turnRate);
        drivetrain.getmDriveStraightPowerController().get
        drivetrain.setArcadeSpeeds(, turnRate);
    }
}
