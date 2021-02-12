package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;
import static frc.robot.Constants.Intake.kIntakeMotor;
import static frc.robot.Constants.Intake.kIntakeToggle;

public class Intake extends SubsystemBase {
    private VictorSPX mIntakeMotor = new VictorSPX(kIntakeMotor);
    private Solenoid mIntakeToggle = new Solenoid(kIntakeToggle);


    public Intake() {
//open loop for the intake motor
        //ability to deploy/retract the intake using solenoid

    }
}
