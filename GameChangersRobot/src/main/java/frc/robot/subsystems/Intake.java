package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kIntakeID;
import static frc.robot.Constants.PneumaticsIDs.kIntakeSolenoidID;

public class Intake extends SubsystemBase {

    private final VictorSPX mIntakeController = new VictorSPX(kIntakeID);
    private final Solenoid mIntakeSolenoid = new Solenoid(kIntakeSolenoidID);

    public Intake() {
        mIntakeController.setInverted(false);

        mIntakeController.setNeutralMode(NeutralMode.Coast);
    }

    public boolean isDeployed() {
        return mIntakeSolenoid.get();
    }

    public void setDeployed(boolean isDeployed) {
        mIntakeSolenoid.set(isDeployed);
    }

    public void setRollerDutyCycle(double targetDutyCycle) {
        mIntakeController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

}
