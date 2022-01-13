package frc.robot.utils.movingAverage;

public interface MovingAverage {

    void clear();

    void addData(double num);

    double getMean();

    int getNumValues();

}