package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax mRightMaster =  new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax mRightSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Drivetrain(){

    }

}


