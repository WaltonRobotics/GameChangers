package frc.robot.utils.interpolation;

import frc.robot.utils.DebuggingLog;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.AdjustableLinearSolver_DDRM;

import java.util.logging.Level;

public class PolynomialRegression {

    private final int mDegree;

    // Vandermonde matrix
    private final DMatrixRMaj mA;
    // matrix containing computed polynomial coefficients
    private final DMatrixRMaj mCoef;
    // observation matrix
    private final DMatrixRMaj mY;

    // solver used to compute
    AdjustableLinearSolver_DDRM mSolver;

    /**
     * Constructor.
     *
     * @param degree The polynomial's degree which is to be fit to the observations.
     */
    public PolynomialRegression(double[][] xy, int degree) {
        this.mDegree = degree;
        this.mCoef = new DMatrixRMaj(degree + 1, 1);
        this.mA = new DMatrixRMaj(1, degree + 1);
        this.mY = new DMatrixRMaj(1, 1);

        // create a solver that allows elements to be added or removed efficiently
        this.mSolver = LinearSolverFactory_DDRM.adjustable();

        fit(xy);
    }

    public PolynomialRegression(double[] x, double[] y, int degree) {
        this.mDegree = degree;
        this.mCoef = new DMatrixRMaj(degree + 1, 1);
        this.mA = new DMatrixRMaj(1, degree + 1);
        this.mY = new DMatrixRMaj(1, 1);

        // create a solver that allows elements to be added or removed efficiently
        this.mSolver = LinearSolverFactory_DDRM.adjustable();

        fit(x, y);
    }

    /**
     * Returns the computed coefficients
     *
     * @return polynomial coefficients that best fit the data.
     */
    public double[] getCoef() {
        return mCoef.data;
    }

    /**
     * Computes the best fit set of polynomial coefficients to the provided observations.
     *
     * @param xy a two-dimensional table of x and y values
     */
    public void fit(double[][] xy) {
        double[] x = new double[xy.length];
        double[] y = new double[xy.length];
        for (int i = 0; i < xy.length; ++i) {
            x[i] = xy[i][0];
            y[i] = xy[i][1];
        }
        fit(x, y);
    }

    /**
     * Computes the best fit set of polynomial coefficients to the provided observations.
     *
     * @param samplePoints where the observations were sampled.
     * @param observations A set of observations.
     */
    public void fit(double[] samplePoints, double[] observations) {
        // Create a copy of the observations and put it into a matrix
        mY.reshape(observations.length, 1, false);
        System.arraycopy(observations, 0, mY.data, 0, observations.length);

        // reshape the matrix to avoid unnecessarily declaring new memory
        // save values is set to false since its old values don't matter
        mA.reshape(mY.numRows, mCoef.numRows, false);

        // set up the A matrix
        for (int i = 0; i < observations.length; i++) {

            double obs = 1;

            for (int j = 0; j < mCoef.numRows; j++) {
                mA.set(i, j, obs);
                obs *= samplePoints[i];
            }
        }

        // process the A matrix and see if it failed
        if (!mSolver.setA(mA))
            throw new RuntimeException("Solver failed");

        // solver the the coefficients
        mSolver.solve(mY, mCoef);
    }

    /**
     * Returns the expected response {@code y} given the value of the predictor variable {@code x}.
     *
     * @param x the value of the predictor variable
     * @return the expected response {@code y} given the value of the predictor variable {@code x}
     */
    public double predict(double x) {
        // horner's method
        double y = 0.0;
        for (int j = mDegree; j >= 0; j--)
            y = getCoef()[j] + (x * y);
        return y;
    }

    /**
     * Removes the observation that fits the model the worst and recomputes the coefficients.
     * This is done efficiently by using an adjustable solver.  Often times the elements with
     * the largest errors are outliers and not part of the system being modeled.  By removing them
     * a more accurate set of coefficients can be computed.
     */
    public void removeWorstFit() {
        // find the observation with the most error
        int worstIndex = -1;
        double worstError = -1;

        for (int i = 0; i < mY.numRows; i++) {
            double predictedObs = 0;

            for (int j = 0; j < mCoef.numRows; j++) {
                predictedObs += mA.get(i, j) * mCoef.get(j, 0);
            }

            double error = Math.abs(predictedObs - mY.get(i, 0));

            if (error > worstError) {
                worstError = error;
                worstIndex = i;
            }
        }

        // nothing left to remove, so just return
        if (worstIndex == -1)
            return;

        DebuggingLog.getInstance().getLogger().log(Level.FINE,
                "Removing observation index " + worstIndex + " from PolynomialRegression");

        // remove that observation
        removeObservation(worstIndex);

        // update A
        mSolver.removeRowFromA(worstIndex);

        // solve for the parameters again
        mSolver.solve(mY, mCoef);
    }

    /**
     * Removes an element from the observation matrix.
     *
     * @param index which element is to be removed
     */
    private void removeObservation(int index) {
        final int N = mY.numRows - 1;
        final double[] d = mY.data;

        // shift
        for (int i = index; i < N; i++) {
            d[i] = d[i + 1];
        }
        mY.numRows--;
    }

    /**
     * Returns the degree of the polynomial to fit.
     *
     * @return the degree of the polynomial to fit
     */
    public int getDegree() {
        return mDegree;
    }

    public double getR2() {
        double SSRes = 0.0;
        double SSTot = 0.0;

        double ySum = 0.0;

        for (int i = 0; i < mY.numRows; i++) {
            ySum += mY.get(i, 0);
        }

        double yAverage = ySum / mY.numRows;

        for (int i = 0; i < mY.numRows; i++) {
            double predictedObs = 0;

            for (int j = 0; j < mCoef.numRows; j++) {
                predictedObs += mA.get(i, j) * mCoef.get(j, 0);
            }

            SSRes += Math.pow(predictedObs - mY.get(i, 0), 2);
            SSTot += Math.pow(predictedObs - yAverage, 2);
        }

        return 1 - SSRes / SSTot;
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder();
        int j = mDegree;

        while (j >= 0 && Math.abs(getCoef()[j]) < 1E-5)
            j--;

        // create remaining terms
        while (j >= 0) {
            if (j == 0)
                s.append(String.format("%.2f ", getCoef()[j]));
            else if (j == 1)
                s.append(String.format("%.2fx + ", getCoef()[j]));
            else
                s.append(String.format("%.2fx^%d + ", getCoef()[j], j));
            j--;
        }

        s.append("  (R^2 = ").append(String.format("%.3f", getR2())).append(")");
        return s.toString();
    }

}
