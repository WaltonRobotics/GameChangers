package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kIntakeMotor;
import static frc.robot.Constants.PneumaticsIDs.kIntakeToggle;

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