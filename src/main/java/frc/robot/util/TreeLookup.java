package frc.robot.util;

import java.util.TreeMap;
import edu.wpi.first.math.MathUtil;

/**
 * A class to allow looking up values in a TreeMap
 * 
 * @author Dhruv Shah
 */
public class TreeLookup {
    private TreeMap<Double, Double> treeMap;
    private double min;
    private double max;

    /**
     * Takes in a TreeMap of generic &lt;Double, Double&gt;
     * 
     * @param treeMap
     */
    public TreeLookup(TreeMap<Double, Double> treeMap) {
        this.treeMap = treeMap;
        this.min = treeMap.values().stream().min(Double::compareTo).orElse(null);
        this.max = treeMap.values().stream().min(Double::compareTo).orElse(null);
    }

    /**
     * Takes an input key and returns an interpolated value
     * 
     * @param input
     * @return interpolation value
     */
    public double getValue(double input) {
        if (treeMap.containsKey(input)) return treeMap.get(input);

        double belowKey = treeMap.floorKey(input); // x1
        double belowValue = treeMap.get(belowKey); // y1
        double aboveKey = treeMap.ceilingKey(input); // x2
        double aboveValue = treeMap.get(aboveKey); // y2

        double slope = (aboveValue - belowValue) / (aboveKey - belowKey); // slope = (y2-y1) / (x2-x1)
        double interpolated = belowValue + (input - belowKey) * slope; // interpolation = y1 + (x-x1) * slope

        return MathUtil.clamp(interpolated, min, max);
    }
}