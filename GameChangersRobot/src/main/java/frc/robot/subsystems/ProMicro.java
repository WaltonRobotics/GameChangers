package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DioIDs.kLEDStripWriteLine1ID;
import static frc.robot.Constants.DioIDs.kLEDStripWriteLine2ID;

public class ProMicro extends SubsystemBase {

    private final DigitalOutput mLEDStripWriteLine1 = new DigitalOutput(kLEDStripWriteLine1ID);
    private final DigitalOutput mLEDStripWriteLine2 = new DigitalOutput(kLEDStripWriteLine2ID);

    public ProMicro() {

    }

    public void setLEDIdleMode() {
        mLEDStripWriteLine1.set(false);
        mLEDStripWriteLine2.set(false);
    }

    public void setLEDTurnLeftMode() {
        mLEDStripWriteLine1.set(true);
        mLEDStripWriteLine2.set(false);
    }

    public void setLEDTurnRightMode() {
        mLEDStripWriteLine1.set(false);
        mLEDStripWriteLine2.set(true);
    }

    public void setLEDAlignedMode() {
        mLEDStripWriteLine1.set(true);
        mLEDStripWriteLine2.set(true);
    }

}
