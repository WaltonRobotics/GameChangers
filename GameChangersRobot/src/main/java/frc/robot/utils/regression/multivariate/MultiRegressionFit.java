package frc.robot.utils.regression.multivariate;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.LinkedHashMap;

// Takes an array of continuous features and fits a multiple linear regression model predicting a named
// feature using all other features.
public class MultiRegressionFit {
    // Fits the model using batch gradient descent with a learning rate specified by the alpha parameter.
    // Cost function is convex so global minimum will always be reached. 
    public static MultiRegressionModel gradientDescent(MultiRegressionObservation[] inputVectors, String dependent, double alpha) {
        // Standardises the input data to have mean 0 and standard deviation 1. 
        Standardisation stanData = standardise(inputVectors);

        // Converts the array of Observation objects to a matrix (2-d array) with an extra first column of 1s for the intercept. 
        double[][] train = new double[inputVectors[0].size() + 1][inputVectors.length];
        for (int i = 0; i < inputVectors.length; i++) {
            train[0][i] = 1; // Intercept
            train[train.length - 1][i] = inputVectors[i].getFeature(dependent); // Dependent variable in the last column
            int j = 1;
            for (String feature : inputVectors[i].getFeatures()) {
                if (!feature.equals(dependent)) {
                    train[j][i] = inputVectors[i].getFeature(feature);
                    j++;
                }
            }
        }

        // One parameter theta per independent variable and one for the intercept. 
        double[] thetas = new double[inputVectors[0].size()];
        double[] temps = new double[thetas.length]; // Parallel array for temp values. 
        double delta; // Absolute change in parameters; used to measure convergence

        do {
            delta = 0;

            // For each parameter theta:
            for (int i = 0; i < thetas.length; i++) {
                // Updates the temp value for that parameter.
                temps[i] = thetas[i] - (alpha * ((double) 1 / train.length) * evaluateCost(thetas, train, i));

                // Calculates the absolute change in that parameter. 
                delta += Math.abs(thetas[i] - temps[i]);
            }

            // Updates each theta to its temp value.
            for (int i = 0; i < thetas.length; i++)
                thetas[i] = temps[i];
        } while (delta > 1E-7); // Threshold at which we conclude that convergence has occurred

        // Destandardises the data and fitted parameters.
        deStandardise(stanData, inputVectors, thetas);

        // Creates a hashmap of the name of each feature and its associated parameter. 
        LinkedHashMap<String, Double> parameters = new LinkedHashMap<String, Double>();
        parameters.put("Intercept", thetas[0]);
        int j = 1;
        for (String feature : inputVectors[1].getFeatures()) {
            if (!feature.equals(dependent)) { // Ignores the dependent variable
                parameters.put(feature, thetas[j]);
                j++;
            }
        }

        // Constructs the Model object. 
        MultiRegressionModel outputModel = new MultiRegressionModel(parameters, dependent, 0);

        // Calculates r-squared and sets it on the model. 
        outputModel.rSquared = calculateRSquared(inputVectors, outputModel);

        return outputModel;
    }

    // Evaluates the cost function (squared error) for a given level of parameters. 
    // Used in the update step of gradient descent. 
    private static double evaluateCost(double[] thetas, double[][] data, int featureIndex) {
        double result = 0;

        // For every row in the dataset:
        for (int i = 0; i < data[0].length; i++) {
            double error = 0;

            // Calculates the value predicted by the parameters.
            for (int j = 0; j < data.length - 1; j++)
                error += data[j][i] * thetas[j];

            // Subtracts the actual value to get the error and scales by the value in that row for the feature being updated. 
            error -= data[data.length - 1][i];
            error *= data[featureIndex][i];

            // Adds the result to the sum. 
            result += error;
        }

        return result;
    }

    // Fits the model in closed form using the normal equation method. 
    public static MultiRegressionModel normalEquation(MultiRegressionObservation[] inputVectors, String dependent) {
        // Builds the design matrix of features from the array of Observation objects, as well as its transpose. 
        double[][] design = new double[inputVectors.length][inputVectors[0].size()];
        double[][] designT = new double[inputVectors[0].size()][inputVectors.length]; // Transpose
        for (int i = 0; i < inputVectors.length; i++) {
            design[i][0] = 1; // Intercept
            designT[0][i] = 1;
            int j = 1;
            for (String feature : inputVectors[i].getFeatures()) {
                if (!feature.equals(dependent)) {
                    design[i][j] = inputVectors[i].getFeature(feature);
                    designT[j][i] = inputVectors[i].getFeature(feature);
                    j++;
                }
            }
        }
        RealMatrix X = new Array2DRowRealMatrix(design);
        RealMatrix XPrime = new Array2DRowRealMatrix(designT);

        // Builds the vector of y values. 
        double[] yArray = new double[inputVectors.length];
        for (int i = 0; i < inputVectors.length; i++)
            yArray[i] = inputVectors[i].getFeature(dependent);
        RealMatrix y = new Array2DRowRealMatrix(yArray);

        // Solves for the parameter vector: theta = (X'X)-1 X'y
        RealMatrix theta = new LUDecomposition(XPrime.multiply(X)).getSolver().getInverse().multiply(XPrime).multiply(y);

        // Creates a hashmap of the name of each feature and its associated fitted parameter. 
        LinkedHashMap<String, Double> parameters = new LinkedHashMap<String, Double>();
        double[] thetas = theta.getColumn(0);
        parameters.put("Intercept", thetas[0]);
        int j = 1;
        for (String feature : inputVectors[1].getFeatures()) {
            if (!feature.equals(dependent)) { // Ignores the dependent variable
                parameters.put(feature, thetas[j]);
                j++;
            }
        }

        // Constructs the Model object. 
        MultiRegressionModel outputModel = new MultiRegressionModel(parameters, dependent, 0);

        // Calculates r-squared and sets it on the model. 
        outputModel.rSquared = calculateRSquared(inputVectors, outputModel);

        return outputModel;
    }

    // Standardises data such that each feature has mean 0 and standard deviation 1.
    // Speeds up convergence of gradient descent.
    private static Standardisation standardise(MultiRegressionObservation[] inputVectors) {
        // Arrays containing the means and standard deviations of each feature.
        double[] xbars = new double[inputVectors[0].size()];
        double[] sigmas = new double[inputVectors[0].size()];

        // Iterates over the input data and adds the value of each feature to the mean.
        for (int i = 0; i < inputVectors.length; i++) {
            int j = 0;
            for (String feature : inputVectors[i].getFeatures()) {
                xbars[j] += inputVectors[i].getFeature(feature);
                j++;
            }
        }

        // Divides each value by the number of observations to yield the mean.
        for (int i = 0; i < xbars.length; i++)
            xbars[i] = xbars[i] / inputVectors.length;

        // Iterates over the input data and adds the squared difference between that value and the mean to the standard deviation.
        for (int i = 0; i < inputVectors.length; i++) {
            int j = 0;
            for (String feature : inputVectors[i].getFeatures()) {
                sigmas[j] += Math.pow(inputVectors[i].getFeature(feature) - xbars[j], 2);
                j++;
            }
        }

        // Square roots and divides each value by the number of observations to yield the standard deviation.
        for (int i = 0; i < sigmas.length; i++)
            sigmas[i] = Math.sqrt(sigmas[i] / inputVectors.length);

        // Iterates over the input data and standardises each value based on feature means and standard deviations.
        for (int i = 0; i < inputVectors.length; i++) {
            int j = 0;
            for (String feature : inputVectors[i].getFeatures()) {
                inputVectors[i].putFeature(feature, (inputVectors[i].getFeature(feature) - xbars[j]) / sigmas[j]);
                j++;
            }
        }

        Standardisation output = new Standardisation(inputVectors, xbars, sigmas);

        return output;
    }

    // Takes a Standardisation object and destandardises data and fitted parameters based on means and standard deviations.
    private static void deStandardise(Standardisation standard, MultiRegressionObservation[] inputVectors, double[] thetas) {
        // Destandardises the intercept and parameters.
        for (int i = 1; i < thetas.length; i++) {
            thetas[0] -= thetas[i] * (standard.xbars[i - 1] / standard.sigmas[i - 1]);
            thetas[i] = (thetas[i] * standard.sigmas[standard.sigmas.length - 1]) / standard.sigmas[i - 1];
        }
        thetas[0] *= standard.sigmas[standard.sigmas.length - 1];
        thetas[0] += standard.xbars[standard.xbars.length - 1];

        // Destandardises the input data.
        for (int i = 0; i < inputVectors.length; i++) {
            int j = 0;
            for (String feature : inputVectors[i].getFeatures()) {
                inputVectors[i].putFeature(feature, ((inputVectors[i].getFeature(feature) * standard.sigmas[j]) + standard.xbars[j]));
                j++;
            }
        }
    }

    // Calculates the R-squared of a fitted model based on the model and the input used to fit it.
    private static double calculateRSquared(MultiRegressionObservation[] inputVectors, MultiRegressionModel model) {
        // Calculates the mean value of y.
        double ybar = 0;
        for (int i = 0; i < inputVectors.length; i++) {
            ybar += inputVectors[i].getFeature(model.dependent);
        }
        ybar /= inputVectors.length;

        // Calculates residual and total sum of squares for the model.
        double rss = 0;
        double tss = 0;
        for (int i = 0; i < inputVectors.length; i++) {
            rss += Math.pow((inputVectors[i].getFeature(model.dependent) - model.predict(inputVectors[i])), 2);
            tss += Math.pow((inputVectors[i].getFeature(model.dependent) - ybar), 2);
        }

        // Returns the R-squared value.
        return (1 - rss / tss);
    }

    // Contains an array of standardised Observation objects, an array of feature means and an array of feature standard deviations.
    // Stores the values used to standardise data so that data and fitted parameters can be destandardised.
    private static class Standardisation {
        public MultiRegressionObservation[] observations;
        public double[] xbars;
        public double[] sigmas;

        public Standardisation(MultiRegressionObservation[] observations, double[] xbars, double[] sigmas) {
            this.observations = observations;
            this.xbars = xbars;
            this.sigmas = sigmas;
        }
    }
}
