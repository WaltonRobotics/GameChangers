package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.IRSensor;
import frc.robot.utils.UtilMethods;

import static frc.robot.Constants.CANBusIDs.kConveyorBackID;
import static frc.robot.Constants.CANBusIDs.kConveyorFrontID;
import static frc.robot.Constants.DioIDs.kConveyorBackSensorID;
import static frc.robot.Constants.DioIDs.kConveyorFrontSensorID;

public class Conveyor extends SubsystemBase {

    private final VictorSPX mConveyorFrontController = new VictorSPX(kConveyorFrontID);
    private final VictorSPX mConveyorBackController = new VictorSPX(kConveyorBackID);

    private final IRSensor mConveyorFrontSensor = new IRSensor(kConveyorFrontSensorID);
    private final IRSensor mConveyorBackSensor = new IRSensor(kConveyorBackSensorID);
    private final EnhancedBoolean mConveyorFrontBool = new EnhancedBoolean();
    private final EnhancedBoolean mConveyorBackBool = new EnhancedBoolean();

    private int mBallCount;

    public Conveyor() {
        mConveyorFrontController.setInverted(true);

        mBallCount = 0;
    }

    public void setFrontDutyCycles(double targetDutyCycles) {
        mConveyorFrontController.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    public void setBackDutyCycles(double targetDutyCycles) {
        mConveyorBackController.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    @Override
    public void periodic() {
        mConveyorFrontSensor.update();
        mConveyorBackSensor.update();

        mConveyorFrontBool.set(mConveyorFrontSensor.get());
        mConveyorBackBool.set(mConveyorBackSensor.get());

        if (mConveyorFrontBool.isRisingEdge()) {
            mBallCount++;
        }

        if (mConveyorBackBool.isFallingEdge()) {
            mBallCount--;
        }

        mBallCount = Math.max(mBallCount, 0);
    }

    public int getBallCount() {
        return mBallCount;
    }
}
