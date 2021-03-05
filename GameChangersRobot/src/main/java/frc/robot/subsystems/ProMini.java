package frc.robot.subsystems;

import frc.robot.utils.DebuggingLog;
import frc.robot.utils.UtilMethods;

import java.util.logging.Level;

import static frc.robot.Constants.ProMiniConstants.kDutyCycleTolerance;

public class ProMini {

    public enum PixyCamReadLineStates {
        NO_DETERMINATION(0.0),
        GALACTIC_SEARCH_RED_A(0.25),
        GALACTIC_SEARCH_RED_B(0.5),
        GALACTIC_SEARCH_BLUE_A(0.75),
        GALACTIC_SEARCH_BLUE_B(1.0);

        private final double mDutyCycle;

        PixyCamReadLineStates(double dutyCycle) {
            this.mDutyCycle = dutyCycle;
        }

        public static PixyCamReadLineStates findByDutyCycle(double dutyCycle) {
            double interval = values()[1].getDutyCycle() - values()[0].getDutyCycle();

            if (kDutyCycleTolerance >= interval / 2) {
                DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                        "Duty cycle tolerance on PixyCam digital read line results in overlapping intervals");
            }

            for (PixyCamReadLineStates state : values()) {
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

    public enum LEDStripWriteLineStates {
        IDLE(0.0),
        TURN_LEFT(0.2),
        TURN_RIGHT(0.4),
        MOVE_FORWARD(0.6),
        MOVE_BACKWARD(0.8),
        ALIGNED_AND_IN_RANGE(1.0);

        private final double mDutyCycle;

        LEDStripWriteLineStates(double dutyCycle) {
            this.mDutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return mDutyCycle;
        }
    }

}
