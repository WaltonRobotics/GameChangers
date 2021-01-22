package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Hardware.*;

public class Drivetrain extends SubsystemBase {
    private final CANSparkMax leftWheelMaster = new CANSparkMax(kLeftMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightWheelMaster = new CANSparkMax(kRightMaster, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftWheelSlave = new CANSparkMax(kLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightWheelSlave = new CANSparkMax(kRightSlave, CANSparkMaxLowLevel.MotorType.kBrushless);

}
