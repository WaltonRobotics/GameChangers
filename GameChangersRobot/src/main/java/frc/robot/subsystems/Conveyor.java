package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kBackConveyorID;
import static frc.robot.Constants.CANBusIDs.kFrontConveyorID;


public class Conveyor extends SubsystemBase {
    private final VictorSPX mFrontConveyorMotor = new VictorSPX(kFrontConveyorID);
    private final VictorSPX mBackConveyorMotor = new VictorSPX(kBackConveyorID);

    public void setFrontDutyCycles(double targetDutyCycles) {
        mFrontConveyorMotor.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    public void setBackDutyCycles(double targetDutyCycles) {
        mBackConveyorMotor.set(ControlMode.PercentOutput, targetDutyCycles);
    }

}
