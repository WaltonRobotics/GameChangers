package frc.robot.commands.background;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.teleop.ToggleClimberDeployCommand;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.SubsystemFlags;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;

import static frc.robot.OI.*;
import static frc.robot.Robot.sClimber;

public class ClimberCommand extends CommandBase {

    private static final double HOLD_DUTY_CYCLE = -0.075;
    private static final double MAX_EXTEND_DUTY_CYCLE = 0.7;
    private static final double DEADBAND = 0.125;

    private final IState mIdle;
    private final IState mClimbing;

    private final StateMachine mStateMachine;

    public ClimberCommand() {
        addRequirements(sClimber);

        mIdle = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                sClimber.setClimberControllerOutput(TalonFXControlMode.PercentOutput, HOLD_DUTY_CYCLE);

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Idle";
            }
        };

        mClimbing = new IState() {
            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                double climbCommand = -sGamepad.getLeftY();

                sClimber.setClimberControllerOutput(PercentOutput, Math.min(climbCommand, MAX_EXTEND_DUTY_CYCLE));

                return determineState();
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Climbing";
            }
        };

        mStateMachine = new StateMachine("Climber", mIdle);

        sLockClimberButton.whenPressed(() -> sClimber.setClimberUnlocked(false));
        sToggleClimberDeployButton.whenPressed(new ToggleClimberDeployCommand());
    }

    @Override
    public void execute() {
        SubsystemFlags.getInstance().setIsClimberDeployed(sClimber.isClimberDeployed());

        mStateMachine.run();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private IState determineState() {
        if (!sClimber.isClimberDeployed()) {
            return mIdle;
        }

        return mClimbing;
    }

}
