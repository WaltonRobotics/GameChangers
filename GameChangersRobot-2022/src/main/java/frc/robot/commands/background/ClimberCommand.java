package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.teleop.ToggleClimberDeployCommand;
import frc.robot.subsystems.SubsystemFlags;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.OI.*;
import static frc.robot.Robot.sClimber;

public class ClimberCommand extends CommandBase {

    private static final double HOLD_DUTY_CYCLE = -0.075;
    private static final double MAX_EXTEND_DUTY_CYCLE = 0.7;
    private static final double DEADBAND = 0.125;

    public ClimberCommand() {
        addRequirements(sClimber);
        sToggleClimberLockButton.whenPressed(() -> sClimber.toggleClimberLock());
        sToggleClimberDeployButton.whenPressed(new ToggleClimberDeployCommand());
    }

    @Override
    public void execute() {
        SubsystemFlags.getInstance().setIsClimberDeployed(sClimber.isClimberDeployed());

        double climbCommand = -sManipulationGamepad.getLeftY();

        if (sClimber.isClimberUnlocked()) {
            if (sClimber.isClimberDeployed() && Math.abs(climbCommand) > DEADBAND) {
                sClimber.setClimberControllerOutput(PercentOutput, Math.min(climbCommand, MAX_EXTEND_DUTY_CYCLE));
            } else {
                sClimber.setClimberControllerOutput(PercentOutput, HOLD_DUTY_CYCLE);
            }
        } else {
            sClimber.setClimberControllerOutput(PercentOutput, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
