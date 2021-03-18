package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfig;
import frc.robot.robots.RobotIdentifier;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import static frc.robot.Constants.CANBusIDs.kIntakeID;
import static frc.robot.Constants.PneumaticsIDs.kDeployIntakeSolenoidID;
import static frc.robot.Constants.PneumaticsIDs.kRetractIntakeSolenoidID;
import static frc.robot.Robot.sCurrentRobot;

public class Intake extends SubsystemBase {

    private final IntakeConfig mConfig = sCurrentRobot.getCurrentRobot().getIntakeConfig();

    private final VictorSPX mIntakeController = new VictorSPX(kIntakeID);

    // On Comp Game Changers, there are two separate solenoids, one for deploying and the other for retracting
    // On Practice Game Changers, there is only one solenoid that manages both actions, so we just use
    // mDeploySolenoid as an alias and set mRetractSolenoid to null in that case
    private final Solenoid mDeploySolenoid = new Solenoid(kDeployIntakeSolenoidID);
    private final Solenoid mRetractSolenoid = sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS ?
            new Solenoid(kRetractIntakeSolenoidID) : null;

    public Intake() {
        mIntakeController.setInverted(false);

        mIntakeController.setNeutralMode(NeutralMode.Coast);
    }

    public boolean isDeployed() {
        return mDeploySolenoid.get();
    }

    public void setDeployed(boolean isDeployed) {
        mDeploySolenoid.set(isDeployed);
    }

    public boolean isRetracted() {
        if (mRetractSolenoid != null) {
            return !mRetractSolenoid.get();
        }

        return false;
    }

    public void setRetracted(boolean isRetracted) {
        if (mRetractSolenoid != null) {
            mRetractSolenoid.set(!isRetracted);
        }
    }

    public void setRollerDutyCycle(double targetDutyCycle) {
        mIntakeController.set(ControlMode.PercentOutput, targetDutyCycle);
    }

    public IntakeConfig getConfig() {
        return mConfig;
    }

    public boolean checkSystem() {
        DebuggingLog.getInstance().getLogger().log(Level.INFO,
                "Checking Intake Subsystem");

        final double kCurrentThres = 0.5;

        mIntakeController.set(VictorSPXControlMode.PercentOutput, 0.0);

        mIntakeController.set(VictorSPXControlMode.PercentOutput, -6.0f / RobotController.getBatteryVoltage());
        Timer.delay(4.0);
        final double intakeCurrent = mIntakeController.getMotorOutputVoltage() / RobotController.getInputCurrent();
        mIntakeController.set(VictorSPXControlMode.PercentOutput, 0.0);

        Timer.delay(2.0);

        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Intake Current: " + intakeCurrent);

        boolean failure = false;

        if (intakeCurrent < kCurrentThres) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Intake Current Low!");
        }

        if (mDeploySolenoid.getPCMSolenoidVoltageFault()) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Deploy solenoid voltage fault!");
        }

        if (mRetractSolenoid != null && mRetractSolenoid.getPCMSolenoidVoltageFault()) {
            failure = true;
            DebuggingLog.getInstance().getLogger().log(Level.WARNING, "Retract solenoid voltage fault!");
        }

        return !failure;
    }

}
