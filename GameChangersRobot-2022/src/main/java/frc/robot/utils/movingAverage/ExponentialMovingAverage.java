package frc.robot.utils.movingAverage;

public class ExponentialMovingAverage implements MovingAverage {

    private final double mAlpha;
    private Double mMostRecentValue;
    private int mNumValues;

    public ExponentialMovingAverage(double alpha) {
        this.mAlpha = alpha;

        clear();
    }

    @Override
    public void clear() {
        mMostRecentValue = null;
        mNumValues = 0;
    }

    @Override
    public void addData(double num) {
        if (mMostRecentValue == null) {
            mMostRecentValue = num;
        }

        mMostRecentValue = mMostRecentValue + mAlpha * (num - mMostRecentValue);
    }

    @Override
    public double getMean() {
        return mMostRecentValue;
    }

    @Override
    public int getNumValues() {
        return mNumValues;
    }

}