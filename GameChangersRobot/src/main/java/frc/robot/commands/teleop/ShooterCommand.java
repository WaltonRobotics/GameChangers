package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateMachine.IState;
import frc.robot.StateMachine.StateMachine;

import static frc.robot.OI.shootButton;
import static frc.robot.Robot.shooterTurret;
import static frc.robot.StateMachine.StateMachine.States.spinningUp;


public class ShooterTurretCommand extends CommandBase {

    IState off = new IState() {
        @Override
        public StateMachine.States initialize() {
            shooterTurret.setOpenLoopDutyCycles(0);
            shootButton.whenPressed(StateMachine.States spinningUp);
            return spinningUp;
        }


        @Override
        public StateMachine.States execute() {
            shootButton.get();
        }

        @Override
        public void finish() {

        }
    }
}
