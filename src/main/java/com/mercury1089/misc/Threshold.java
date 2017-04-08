package com.mercury1089.misc;

/**
 * Simple class for holding threshold values,
 * as well as a simple check to see if values are in a threshold.
 * Thresholds are inclusive ranges, meaning some value x must be:
 * <pre>min \u2264 x \u2264 max</pre>
 * in order to be within the threshold.
 */
public class Threshold {
    private double min, max;

    /**
     * Creates a new {@code Threshold} with the specified min and max
     * @param a the min
     * @param b the max
     */
    public Threshold(double a, double b) {
        min = a;
        max = b;
    }

    public void setMin(double v) {
        min = v;
    }

    public void setMax(double v) {
        max = v;
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    /**
     * Sets the min and max for the threshold.
     * @param a the min
     * @param b the max
     */
    public void setThreshold(double a, double b) {
        setMin(a);
        setMax(b);
    }

    public double[] toArray() {
        return new double[]{min, max};
    }

    /**
     * Check if the value is within the range.
     * The range is inclusive, including the endpoints.
     *
     * @param val the value to test against the threshold
     *
     * @return whether or not min \u2264 val \u2264 max
     */
    public boolean isInThreshold(double val) {
        return min <= val && val <= max;
    }
}
