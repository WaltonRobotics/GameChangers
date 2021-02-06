package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateMachine.IState;

import static frc.robot.OI.shootButton;
import static frc.robot.Robot.shooter;
import static frc.robot.StateMachine.StateMachine.States.spinningUp;


public class ShooterCommand
extends CommandBase {

    IState off = new IState() {
        @Override
        public void initialize() {
            shooter.setOpenLoopDutyCycles(0);
            shootButton.whenPressed(return spinningUp)
        }
    }

    @Override
    public IState execute() {
        shootButton.get();
    }

    @Override
    public void finish() {

    }
}
