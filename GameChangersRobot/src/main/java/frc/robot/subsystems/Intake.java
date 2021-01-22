package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private VictorSPX mIntakeMotor = new VictorSPX(5);
    private Solenoid mIntakeToggle = new Solenoid(1);
    private VictorSPX mFrontConveyorMotor = new VictorSPX(7);
    private VictorSPX mBackConveyorMotor = new VictorSPX(8);
}
