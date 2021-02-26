package frc.robot.utils.movingAverage;

public interface MovingAverage {

    void addData(double num);

    double getMean();

}
