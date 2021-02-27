package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import java.util.Arrays;
import frc.robot.utils.Util;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
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

    public boolean checkSystem() {
        final double kCurrentThres = 0.5;

        mIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0 / getBatteryVoltage());

        mIntakeMotor.set(VictorSPXControlMode.PercentOutput, -6.0f / getBatteryVoltage());
        Timer.delay(4.0);
        final double currentMaster = mIntakeMotor.getMotorOutputVoltage() / RobotController.getInputCurrent();
        mIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0 / getBatteryVoltage());

        Timer.delay(2.0);


        System.out.println("Intake Master Current: " + currentMaster);

        boolean failure = false;

        if (currentMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!! Intake Master Current Low !!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentMaster), currentMaster, 5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!!! Intake Currents different !!!!!!!!!!!!!!!");
        }

        return !failure;
    }

}
