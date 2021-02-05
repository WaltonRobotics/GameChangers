package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.*;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax mRightMaster =  new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax mRightSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax mLeftMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax mLeftSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Drivetrain(){

    }


}


