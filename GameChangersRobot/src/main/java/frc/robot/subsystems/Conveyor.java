package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Util;

import java.util.Arrays;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;
import static frc.robot.OI.mFrontConveyorMotor;

public class Conveyor extends SubsystemBase {
    private VictorSPX mFrontConveyorMotor = new VictorSPX(kFrontConveyorMotor);
    private VictorSPX mBackConveyorMotor = new VictorSPX(kBackConveyorMotor);

    public void setFrontOpenLoopDutyCycles(double targetDutyCycles) {
        mFrontConveyorMotor.set(ControlMode.Velocity, targetDutyCycles);
    }

    public void setFrontClosedLoopVelocity(double targetVelocity) {
        mFrontConveyorMotor.set(ControlMode.Velocity, targetVelocity);
    }

    public void setBackOpenLoopDutyCycles(double targetDutyCycles) {
        mBackConveyorMotor.set(ControlMode.Velocity, targetDutyCycles);
    }

    public void setBackClosedLoopVelocity(double targetVelocity) {
        mBackConveyorMotor.set(ControlMode.Velocity, targetVelocity);
    }
    public boolean checkSystem() {
        System.out.println("Testing Conveyor.-----------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThes = 2000.0;

        mBackConveyorMotor.changeControlMode(VictorSPXControlMode.);
        mFrontConveyorMotor.changeControlMode(VictorSPXControlMode.Voltage);

        mBackConveyorMotor.set(0.0);
        mFrontConveyorMotor.set(0.0);

        mFrontConveyorMotor.set(6.0f);
        Timer.delay(4.0);
        final double currentMaster = mFrontConveyorMotor.getOutputCurrent();
        final double rpmMaster = mFrontConveyorMotor.getSpeed();
        mFrontConveyorMotor.set(0.0f);

        Timer.delay(2.0);

        mBackConveyorMotor.set(-6.0f);
        Timer.delay(4.0);
        final double currentSlave = mBackConveyorMotor.getOutputCurrent();
        final double rpmSlave = mFrontConveyorMotor.getSpeed();
        mBackConveyorMotor.set(0.0f);

        mBackConveyorMotor.changeControlMode(TalonControlMode.Follower);
        mBackConveyorMotor.set(Constants.kConveyorMasterId);

        System.out.println("Conveyor Master Current: " + currentMaster + " Slave Current: " + currentSlave
                + " rpmMaster: " + rpmMaster + " rpmSlave: " + rpmSlave);

        boolean failure = false;

        if (currentMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!! Conveyor Master Current Low !!!!!!!!!!!!!!!!");
        }

        if (currentSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!! Conveyor Slave Current Low !!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentMaster, currentSlave), currentMaster, 5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!! Conveyor currents different!!!!!!!!!!!!!!!");
        }

        if (rpmMaster < kRpmThes) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!! Conveyor Master RPM Low !!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (rpmSlave < kRpmThes) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!! Conveyor Slave RPM Low !!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmMaster, rpmSlave), rpmMaster, 250)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!! Conveyor RPM different !!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }

}