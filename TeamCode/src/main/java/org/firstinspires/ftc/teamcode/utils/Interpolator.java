package org.firstinspires.ftc.teamcode.utils;

import java.util.TreeMap;
import java.util.Map;

public class Interpolator {
    private final TreeMap<Double, Double> table = new TreeMap<>();

    //table store distance and rpm
    public void add(double distance, double rpm) {
        table.put(distance, rpm);
    }

    public double getRPM(double distance) {
        if (table.isEmpty()) return 0;

        // The two distances far and shorter than current one
        Map.Entry<Double, Double> floor = table.floorEntry(distance);
        Map.Entry<Double, Double> ceiling = table.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue(); // shorter than floor
        if (ceiling == null) return floor.getValue(); // far than ceiling
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        // formula for Linear interpolation
        double d1 = floor.getKey();
        double d2 = ceiling.getKey();
        double v1 = floor.getValue();
        double v2 = ceiling.getValue();

        return v1 + (distance - d1) * (v2 - v1) / (d2 - d1);
    }

}
