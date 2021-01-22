package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private TalonFX mFlyWheelMaster = new TalonFX(9);
    private TalonFX mFlyWheelSlave = new TalonFX(10);

}
