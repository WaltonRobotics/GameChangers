package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConveyorConfig;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.IRSensor;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
import static frc.robot.Constants.CANBusIDs.kBackConveyorID;
import static frc.robot.Constants.CANBusIDs.kFrontConveyorID;
import static frc.robot.Constants.Conveyor.kFrontLoadingCapacity;
import static frc.robot.Constants.Conveyor.kMaximumBallCapacity;
import static frc.robot.Constants.DioIDs.kConveyorBackSensorID;
import static frc.robot.Constants.DioIDs.kConveyorFrontSensorID;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Robot.sCurrentRobot;

public class Conveyor extends SubsystemBase {

    private final ConveyorConfig mConfig = sCurrentRobot.getCurrentRobot().getConveyorConfig();

    private final VictorSPX mFrontConveyorController = new VictorSPX(kFrontConveyorID);
    private final VictorSPX mBackConveyorController = new VictorSPX(kBackConveyorID);

    private final IRSensor mFrontConveyorSensor = new IRSensor(kConveyorFrontSensorID, mConfig.kIRSensorFlickeringTimeSeconds);
    private final IRSensor mBackConveyorSensor = new IRSensor(kConveyorBackSensorID, mConfig.kIRSensorFlickeringTimeSeconds);
    private final EnhancedBoolean mFrontConveyorBool = new EnhancedBoolean();
    private final EnhancedBoolean mBackConveyorBool = new EnhancedBoolean();

    private int mBallCount;

    public Conveyor() {
        mFrontConveyorController.setInverted(true);

        resetBallCount();
    }

    public void setFrontDutyCycle(double targetDutyCycle) {
        mFrontConveyorController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

    public void setFrontVoltage(double targetVoltage) {
        setFrontDutyCycle(targetVoltage / getBatteryVoltage());
    }

    public void setBackDutyCycle(double targetDutyCycle) {
        mBackConveyorController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

    public void setBackVoltage(double targetVoltage) {
        setBackDutyCycle(targetVoltage / getBatteryVoltage());
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

        SmartDashboard.putBoolean(kConveyorFrontSensorStateKey, mFrontConveyorBool.get());
        SmartDashboard.putBoolean(kConveyorBackSensorStateKey, mBackConveyorBool.get());
        SmartDashboard.putNumber(kConveyorBallCountKey, mBallCount);
    }

    public void resetBallCount() {
        setBallCount(0);
    }

    public void setBallCount(int ballCount) {
        mBallCount = ballCount;
    }

    public int getBallCount() {
        return mBallCount;
    }

    public boolean shouldNudge() {
        return getBallCount() < kMaximumBallCapacity - kFrontLoadingCapacity && mFrontConveyorBool.get();
    }

    public ConveyorConfig getConfig() {
        return mConfig;
    }
}
