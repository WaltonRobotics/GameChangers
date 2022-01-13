package frc.robot.utils;

import java.util.logging.Level;
import java.util.logging.Logger;

public class DebuggingLog {

    private static final DebuggingLog sInstance = new DebuggingLog();

    private final Logger mLogger = Logger.getLogger("frc.robot");

    public DebuggingLog() {
        mLogger.setLevel(Level.FINEST);
    }

    public static DebuggingLog getInstance() {
        return sInstance;
    }

    public Logger getLogger() {
        return mLogger;
    }

}
