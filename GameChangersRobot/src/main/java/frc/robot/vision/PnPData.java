package frc.robot.vision;

import frc.robot.utils.movingAverage.MovingAverage;
import frc.robot.utils.movingAverage.SimpleMovingAverage;

public class PnPData {

    public MovingAverage xInchesMovingAverage;
    public MovingAverage yInchesMovingAverage;
    public MovingAverage zInchesMovingAverage;
    public MovingAverage pitchDegreesMovingAverage;
    public MovingAverage yawDegreesMovingAverage;
    public MovingAverage rollDegreesMovingAverage;

    public PnPData(int windowSize) {
        xInchesMovingAverage = new SimpleMovingAverage(windowSize);
        yInchesMovingAverage = new SimpleMovingAverage(windowSize);
        zInchesMovingAverage = new SimpleMovingAverage(windowSize);
        pitchDegreesMovingAverage = new SimpleMovingAverage(windowSize);
        yawDegreesMovingAverage = new SimpleMovingAverage(windowSize);
        rollDegreesMovingAverage = new SimpleMovingAverage(windowSize);
    }

    public double getXInches() {
        return xInchesMovingAverage.getMean();
    }

    public double getYInches() {
        return yInchesMovingAverage.getMean();
    }

    public double getZInches() {
        return zInchesMovingAverage.getMean();
    }

    public double getPitchDegrees() {
        return pitchDegreesMovingAverage.getMean();
    }

    public double getYawDegrees() {
        return yawDegreesMovingAverage.getMean();
    }

    public double getRollDegrees() {
        return rollDegreesMovingAverage.getMean();
    }

}
