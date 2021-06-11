package frc.robot.commands.teleop;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Robot.sClimber;

public class UnlockClimberCommand extends CommandBase {

    private static final double KICK_BACK_DUTY_CYCLE = -0.5;
    private static final double KICK_BACK_TIME = 0.1;

    private double startTime;

    public UnlockClimberCommand() {
        addRequirements(sClimber);
    }

    @Override
    public void initialize() {
        sClimber.setClimberUnlocked(true);
        startTime = getFPGATimestamp();
    }

    @Override
    public void execute() {
        sClimber.setClimberControllerOutput(TalonFXControlMode.PercentOutput, KICK_BACK_DUTY_CYCLE);
    }

    @Override
    public void end(boolean interrupted) {
        sClimber.setClimberControllerOutput(PercentOutput, 0);
    }

    @Override
    public boolean isFinished() {
        return getFPGATimestamp() - startTime > KICK_BACK_TIME;
    }

}
