package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;

import java.util.logging.Level;

import static frc.robot.Constants.DioIDs.kLEDStripWriteLineID;
import static frc.robot.Constants.DioIDs.kPixyCamReadLineID;
import static frc.robot.Constants.ProMini.kDutyCycleTolerance;

public class ProMini extends SubsystemBase {

    private final DigitalInput mPixyCamReadLine = new DigitalInput(kPixyCamReadLineID);
    private final DutyCycle mPixyCamReadLineDutyCycle = new DutyCycle(mPixyCamReadLine);

    private final DigitalOutput mLEDStripWriteLine = new DigitalOutput(kLEDStripWriteLineID);
    private final DutyCycle mLEDStripWriteLineDutyCycle = new DutyCycle(mLEDStripWriteLine);

    public enum PixyCamReadLineState {
        NO_DETERMINATION(0.0),
        GALACTIC_SEARCH_RED_A(0.25),
        GALACTIC_SEARCH_RED_B(0.5),
        GALACTIC_SEARCH_BLUE_A(0.75),
        GALACTIC_SEARCH_BLUE_B(1.0);

        private final double mDutyCycle;

        PixyCamReadLineState(double dutyCycle) {
            this.mDutyCycle = dutyCycle;
        }

        public static PixyCamReadLineState findByDutyCycle(double dutyCycle) {
            double interval = values()[1].getDutyCycle() - values()[0].getDutyCycle();

            if (kDutyCycleTolerance >= interval / 2) {
                DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                        "Duty cycle tolerance on PixyCam digital read line results in overlapping intervals");
            }

            for (PixyCamReadLineState state : values()) {
                if (UtilMethods.isWithinTolerance(dutyCycle, state.getDutyCycle(), 0.5)) {
                    return state;
                }
            }

            return NO_DETERMINATION;
        }

        public double getDutyCycle() {
            return mDutyCycle;
        }
    }

    public enum LEDStripWriteLineState {
        IDLE(0.0),
        TURN_LEFT(0.2),
        TURN_RIGHT(0.4),
        MOVE_FORWARD(0.6),
        MOVE_BACKWARD(0.8),
        ALIGNED_AND_IN_RANGE(1.0);

        private final double mDutyCycle;

        LEDStripWriteLineState(double dutyCycle) {
            this.mDutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return mDutyCycle;
        }
    }

    public ProMini() {
        mLEDStripWriteLine.enablePWM(0.0);
    }

    public void setLEDStripState(LEDStripWriteLineState state) {
        mLEDStripWriteLine.updateDutyCycle(state.getDutyCycle());
    }

    public PixyCamReadLineState getPixyCamState() {
        return PixyCamReadLineState.findByDutyCycle(mPixyCamReadLineDutyCycle.getOutput());
    }

}
