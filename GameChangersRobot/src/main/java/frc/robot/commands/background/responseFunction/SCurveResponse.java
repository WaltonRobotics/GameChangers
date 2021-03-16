package frc.robot.commands.background.responseFunction;

public class SCurveResponse implements ResponseFunction {

    @Override
    public double getOutput(double input) {
        if (input < 0) {
            return -3 * (1 + input) * Math.pow(input, 2) + Math.pow(input, 3);
        }

        return 3 * (1 - input) * Math.pow(input, 2) + Math.pow(input, 3);
    }

}
