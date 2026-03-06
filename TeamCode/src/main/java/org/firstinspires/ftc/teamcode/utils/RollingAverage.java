package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayDeque;
import java.util.Queue;

public class RollingAverage {
    private static final int MAX_SIZE = 20;

    private Queue<Double> readings = new ArrayDeque<>();
    private double sum = 0.0;

    public void reset()
    {
        sum=0.0;
        readings.clear();
    }

    public double addReading(double value) {
        readings.add(value);
        sum += value;

        if (readings.size() > MAX_SIZE) {
            sum -= readings.poll(); // remove oldest value
        }

        return getAverage();
    }

    public double getAverage() {
        if (readings.isEmpty()) return 0;
        return sum / readings.size();
    }


}