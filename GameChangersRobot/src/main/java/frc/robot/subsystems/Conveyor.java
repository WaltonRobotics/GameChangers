package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConveyorConfig;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.EnhancedBoolean;
import frc.robot.utils.IRSensor;
import frc.robot.utils.UtilMethods;

import java.util.Arrays;
import java.util.logging.Level;

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
        mFrontConveyorController.setInverted(mConfig.kIsFrontConveyorControllerInverted);
        mBackConveyorController.setInverted(mConfig.kIsBackConveyorControllerInverted);

        resetBallCount();
    }

    public void setFrontDutyCycle(double targetDutyCycle) {
        mFrontConveyorController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

    public void setFrontVoltage(double targetVoltage) {
        setFrontDutyCycle(targetVoltage / RobotController.getBatteryVoltage());
    }

    public void setBackDutyCycle(double targetDutyCycle) {
        mBackConveyorController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

    public void setBackVoltage(double targetVoltage) {
        setBackDutyCycle(targetVoltage / RobotController.getBatteryVoltage());
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

    public boolean checkSystem() {
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Checking Conveyor Subsystem");

        final double kCurrentThres = 0.5;

        mBackConveyorController.set(VictorSPXControlMode.PercentOutput, 0.0);
        mFrontConveyorController.set(VictorSPXControlMode.PercentOutput, 0.0);

        mFrontConveyorController.set(VictorSPXControlMode.PercentOutput, 6.0f / RobotController.getBatteryVoltage());
        Timer.delay(4.0);
        final double frontConveyorCurrent = mFrontConveyorController.getMotorOutputVoltage()
                / RobotController.getInputCurrent();
        mFrontConveyorController.set(VictorSPXControlMode.PercentOutput, 0.0);

        Timer.delay(2.0);

        mBackConveyorController.set(VictorSPXControlMode.PercentOutput, -6.0f / RobotController.getBatteryVoltage());
        Timer.delay(4.0);
        final double backConveyorCurrent = mBackConveyorController.getMotorOutputVoltage()
                / RobotController.getInputCurrent();
        mBackConveyorController.set(VictorSPXControlMode.PercentOutput, 0.0f);

        DebuggingLog.getInstance().getLogger().log(Level.INFO,"Front Conveyor Current: " + frontConveyorCurrent
                + " Back Conveyor Current: " + backConveyorCurrent);

        boolean failure = false;

        if (frontConveyorCurrent < kCurrentThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Front Conveyor Current Low!");
        }

        if (backConveyorCurrent < kCurrentThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Back Conveyor Current Low!");
        }

        if (!UtilMethods.allWithinTolerance(Arrays.asList(frontConveyorCurrent, backConveyorCurrent),
                frontConveyorCurrent, 5.0)) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                    "Conveyor Currents Different!");
        }

        return !failure;
    }
}
