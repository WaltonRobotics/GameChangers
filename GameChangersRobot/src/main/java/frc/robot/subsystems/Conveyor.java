package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IRSensor;

import static frc.robot.Constants.CANBusIDs.kBackConveyorID;
import static frc.robot.Constants.CANBusIDs.kFrontConveyorID;
import static frc.robot.Constants.DioIDs.kBackConveyorSensorID;
import static frc.robot.Constants.DioIDs.kFrontConveyorSensorID;


public class Conveyor extends SubsystemBase {

    private final VictorSPX mFrontConveyorMotor = new VictorSPX(kFrontConveyorID);
    private final VictorSPX mBackConveyorMotor = new VictorSPX(kBackConveyorID);

    private final IRSensor mFrontConveyorSensor = new IRSensor(kFrontConveyorSensorID);
    private final IRSensor mBackConveyorSensor = new IRSensor(kBackConveyorSensorID);

    public Conveyor() {
        mFrontConveyorMotor.setInverted(true);
    }

    public void setFrontDutyCycles(double targetDutyCycles) {
        mFrontConveyorMotor.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    public void setBackDutyCycles(double targetDutyCycles) {
        mBackConveyorMotor.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    @Override
    public void periodic() {
        mFrontConveyorSensor.update();
        mBackConveyorSensor.update();
    }
}
