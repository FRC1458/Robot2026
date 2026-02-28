package frc.robot.lib.util;

import java.util.LinkedList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverageDouble {
    private LinkedList<Double> current = new LinkedList<Double>();
    private int maxSize;

    /**
     * Generates a MovingAverage object
     * @param maxSize The maximum size of the moving average.
     * @param zero The zero element of type T.
     * @param add The addition function of type T.
     * @param divide The scalar division function of type T.
     */
    public MovingAverageDouble(int maxSize) {
        this.maxSize = maxSize;
    }

    /**
     * Adds a value to the moving average, popping the first value if it exceeds the max size.
     * @param other The value to add.
     */
    public void add(double other) {
        current.add(other);
        if (current.size() > maxSize) {
            current.removeFirst();
        }
    }

    /**
     * Gets the moving average.
     * @return The moving average.
     */
    public double getAverage() {
        double total = 0;

        for (double x : current) {
            total += x;
        }

        return total / current.size();
    }

    /**
     * Gets the current size of the moving average.
     * @return The current size.
     */
    public int getSize() {
        return current.size();
    }

    /**
     * Whether the moving average is under max capacity.
     * @return Whether the moving average is under max capacity.
     */
    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    /**
     * Clears the current moving average.
     */
    public void clear() {
        current.clear();
    }
}