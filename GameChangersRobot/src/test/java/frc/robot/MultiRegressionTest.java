package frc.robot;

import frc.robot.utils.regression.multivariate.MultiRegressionFit;
import frc.robot.utils.regression.multivariate.MultiRegressionModel;
import frc.robot.utils.regression.multivariate.MultiRegressionObservation;
import org.junit.Assert;
import org.junit.Test;

public class MultiRegressionTest {

    @Test
    public void checkMultiRegression() {
        // Dataset from http://statland.org/Software_Help/R/R2v_reg.htm
        double[][] observationsTable = new double[][] {
                { 1.350, 6, 8 },
                { 1.960, 6, 8 },
                { 2.270, 6, 9 },
                { 2.483, 6, 10 },
                { 2.730, 6, 11 },
                { 3.091, 7, 11 },
                { 3.647, 7, 12 },
                { 4.620, 7, 16 },
                { 5.497, 7, 18 },
                { 6.260, 8, 19 },
                { 7.012, 8, 20 },
                { 7.618, 8, 21 },
                { 8.131, 8, 22 },
                { 8.593, 8, 23 },
        };

        MultiRegressionObservation[] observationList
                = new MultiRegressionObservation[observationsTable.length];

        for (int i = 0; i < observationList.length; i++) {
            double[] data = observationsTable[i];

            MultiRegressionObservation observation = new MultiRegressionObservation();

            observation.putFeature("Licenses", data[0]);
            observation.putFeature("Lengths", data[1]);
            observation.putFeature("Defects", data[2]);

            observationList[i] = observation;
        }

        double alpha = 0.01;

        MultiRegressionModel gradientFit = MultiRegressionFit.gradientDescent(
                observationList,
                "Defects",
                alpha
        );

        MultiRegressionModel normalFit = MultiRegressionFit.normalEquation(
                observationList,
                "Defects"
        );

        System.out.println("Gradient fit model:");
        System.out.println(gradientFit);

        System.out.println("Normal fit model:");
        System.out.println(normalFit);

        MultiRegressionObservation validationInputVector = observationList[5];

        System.out.print("Gradient fit validation prediction: ");
        System.out.println(gradientFit.predict(validationInputVector));
        System.out.print("Normal fit validation prediction: ");
        System.out.println(normalFit.predict(validationInputVector));
        System.out.println("Expected value: " + validationInputVector.getFeature("Defects"));

        Assert.assertTrue(gradientFit.rSquared > 0.85);
        Assert.assertTrue(normalFit.rSquared > 0.85);
    }

}
