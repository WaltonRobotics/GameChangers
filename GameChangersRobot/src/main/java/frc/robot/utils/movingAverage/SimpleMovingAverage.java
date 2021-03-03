package frc.robot.utils.movingAverage;

import java.util.LinkedList;
import java.util.Queue;

public class SimpleMovingAverage implements MovingAverage {

    // queue used to store list so that we get the average
    private final Queue<Double> mDataset = new LinkedList<Double>();
    private final int mPeriod;
    private double mSum;
    private int mNumValues;

    // constructor to initialize period
    public SimpleMovingAverage(int period) {
        this.mPeriod = period;
    }

    @Override
    public void clear() {
        mDataset.clear();
        mSum = 0;
        mNumValues = 0;
    }

    // function to add new data in the
    // list and update the sum so that
    // we get the new mean
    @Override
    public void addData(double num) {
        mSum += num;
        mDataset.add(num);

        // Updating size so that length
        // of data set should be equal
        // to period as a normal mean has
        if (mDataset.size() > mPeriod) {
            mSum -= mDataset.remove();
        }

        mNumValues++;
    }

    // function to calculate mean
    @Override
    public double getMean() {
        return mSum / mPeriod;
    }

    @Override
    public int getNumValues() {
        return mNumValues;
    }

}