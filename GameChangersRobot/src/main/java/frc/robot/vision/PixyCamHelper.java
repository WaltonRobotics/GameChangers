package frc.robot.vision;

import frc.robot.subsystems.ProMicro;

import static frc.robot.Robot.sProMicro;

public class PixyCamHelper {

    public static ProMicro.PixyCamReadLineState getGalacticSearchDetermination() {
        return sProMicro.getPixyCamState();
    }

}
