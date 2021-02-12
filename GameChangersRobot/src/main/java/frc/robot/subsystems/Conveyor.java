package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import static frc.robot.Constants.Conveyor.kBackConveyorMotor;
import static frc.robot.Constants.Conveyor.kFrontConveyorMotor;

public class Conveyor {
    private VictorSPX mFrontConveyorMotor = new VictorSPX(kFrontConveyorMotor);
    private VictorSPX mBackConveyorMotor = new VictorSPX(kBackConveyorMotor);
}
