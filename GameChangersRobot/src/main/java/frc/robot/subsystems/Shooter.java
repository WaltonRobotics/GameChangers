package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.OI.mFlyWheelMaster;

public class Shooter extends SubsystemBase {

    public Shooter(double targetVelocity) {


        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20, 10);

    }
    private void setOpenLoopDutyCycles(double targetDutyCycles) {

    }

    private void setClosedLoopVelocity (double targetVelocity) {
        mFlyWheelMaster.set(ControlMode.Velocity, targetVelocity);
    }

    private double getVelocity(){
    }
}