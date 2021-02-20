package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;
import static frc.robot.OI.mFlyWheelMaster;

public class Conveyor {
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


}
