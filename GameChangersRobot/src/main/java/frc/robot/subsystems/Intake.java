package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;
import static frc.robot.Constants.Intake.kIntakeMotor;
import static frc.robot.Constants.Intake.kIntakeToggle;

public class Intake extends SubsystemBase {
    private VictorSPX mIntakeMotor = new VictorSPX(kIntakeMotor);
    private Solenoid mIntakeToggle = new Solenoid(kIntakeToggle);


    public Intake() {

    }

    public void setIntakeDeployed(boolean isDeployed) {
        mIntakeToggle.set(isDeployed);
    }

    public boolean isDeployed() {
        return mIntakeToggle.get();
    }

    public void setRollerDutyCycles(double targetDutyCycles) {
        mIntakeMotor.set(ControlMode.PercentOutput, targetDutyCycles);
    }


}
