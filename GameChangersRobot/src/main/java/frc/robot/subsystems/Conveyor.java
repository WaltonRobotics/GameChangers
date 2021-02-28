package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.IRSensor;

import static frc.robot.Constants.CANBusIDs.kBackConveyorID;
import static frc.robot.Constants.CANBusIDs.kFrontConveyorID;
import static frc.robot.Constants.Conveyor.kMaximumBallCapacity;
import static frc.robot.Constants.DioIDs.kConveyorBackSensorID;
import static frc.robot.Constants.DioIDs.kConveyorFrontSensorID;

public class Conveyor extends SubsystemBase {

    private final VictorSPX mFrontConveyorController = new VictorSPX(kFrontConveyorID);
    private final VictorSPX mBackConveyorController = new VictorSPX(kBackConveyorID);

    private final IRSensor mFrontConveyorSensor = new IRSensor(kConveyorFrontSensorID);
    private final IRSensor mBackConveyorSensor = new IRSensor(kConveyorBackSensorID);
    private final EnhancedBoolean mFrontConveyorBool = new EnhancedBoolean();
    private final EnhancedBoolean mBackConveyorBool = new EnhancedBoolean();

    private int mBallCount;

    public Conveyor() {
        mFrontConveyorController.setInverted(true);

        mBallCount = 0;
    }

    public void setFrontDutyCycles(double targetDutyCycles) {
        mFrontConveyorController.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    public void setBackDutyCycles(double targetDutyCycles) {
        mBackConveyorController.set(ControlMode.PercentOutput, targetDutyCycles);
    }

    @Override
    public void periodic() {
        mFrontConveyorSensor.update();
        mBackConveyorSensor.update();

        mFrontConveyorBool.set(mFrontConveyorSensor.get());
        mBackConveyorBool.set(mBackConveyorSensor.get());

        if (mFrontConveyorBool.isRisingEdge()) {
            mBallCount++;
        }

        if (mBackConveyorBool.isFallingEdge()) {
            mBallCount--;
        }

        mBallCount = Math.max(mBallCount, 0);
    }

    public int getBallCount() {
        return mBallCount;
    }

    public boolean shouldNudge() {
        return getBallCount() < kMaximumBallCapacity - 1 && mFrontConveyorBool.get();
    }
}