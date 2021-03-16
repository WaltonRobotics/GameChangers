package frc.robot.vision;

import frc.robot.subsystems.ProMicro;

import static frc.robot.Robot.sProMicro;

public class PixyCamHelper {

    public static ProMicro.PixyCamReadMessage getGalacticSearchDetermination() {
        return sProMicro.getPixyCamDetermination();
    }

}
