package frc.robot.utils.movingAverage;

public class ExponentialMovingAverage implements MovingAverage {

    private double mAlpha;
    private Double mMostRecentValue;

    public ExponentialMovingAverage(double alpha) {
        this.mAlpha = alpha;
        this.mMostRecentValue = null;
    }

    @Override
    public void addData(double num) {
        if (mMostRecentValue == null) {
            mMostRecentValue = num;
        }

        double newValue = mMostRecentValue + mAlpha * (num - mMostRecentValue);
        mMostRecentValue = newValue;
    }

    @Override
    public double getMean() {
        return mMostRecentValue;
    }

}
