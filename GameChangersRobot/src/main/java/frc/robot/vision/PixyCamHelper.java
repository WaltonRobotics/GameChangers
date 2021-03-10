package frc.robot.vision;

import frc.robot.subsystems.ProMini;

import static frc.robot.Robot.sProMini;

public class PixyCamHelper {

    public static ProMini.PixyCamReadLineState getGalacticSearchDetermination() {
        return sProMini.getPixyCamState();
    }

}
