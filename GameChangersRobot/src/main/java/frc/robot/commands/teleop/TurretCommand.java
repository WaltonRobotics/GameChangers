package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Turret;

import static frc.robot.Robot.turret;

public class TurretCommand extends CommandBase {
    public TurretCommand() {addRequirements(turret);}

    IState mManual = new IState() {
        @Override
        public void initialize() {

        }

        @Override
        public IState execute() {
            return null;
        }

        @Override
        public void finish() {

        }
    }

}
