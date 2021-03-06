package frc.robot;

import frc.robot.utils.interpolation.PolynomialRegression;
import org.junit.Assert;
import org.junit.Test;

public class PolynomialRegressionTest {

    @Test
    public void checkRegression() {
        // From years of experience vs. salary dataset:
        // https://s3.us-west-2.amazonaws.com/public.gamelab.fun/dataset/position_salaries.csv
        double[][] xy = {
                { 1, 45000 },
                { 2, 50000 },
                { 3, 60000 },
                { 4, 80000 },
                { 5, 110000 },
                { 6, 150000 },
                { 7, 200000 },
                { 8, 300000 },
                { 9, 500000 },
                { 10, 1000000 },
        };

        PolynomialRegression polynomial = new PolynomialRegression(xy, 2);

        System.out.println(polynomial);
        System.out.println(polynomial.predict(9.5));

        Assert.assertTrue(polynomial.getR2() > 0.8);
    }

}
