package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateMachine.IState;

import static frc.robot.OI.shootButton;
import static frc.robot.Robot.shooter;
import static frc.robot.StateMachine.StateMachine.States.spinningUp;

import frc.robot.StateMachine.StateMachine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.MovingAverage;

public class ShooterCommand extends CommandBase {

    double targetVelocity = 1.5;
    double tolerance = 1;

    IState off = new IState() {
        @Override
        public void initialize() {
            shooter.setOpenLoopDutyCycles(0);
        }


        @Override
        public IState execute() {
            if(shootButton.get()){

                if(shooter.getAverageClosedLoopVelocity() - targetVelocity <= tolerance)


            }
        }

        @Override
        public void finish() {

        }
    }
}
