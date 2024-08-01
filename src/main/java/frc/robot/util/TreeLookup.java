package frc.robot.util;

import java.util.TreeMap;

/**
 * The TreeLookup class takes in a TreeMap of generic &lt;Double, Double&gt; in
 * the constructor and allows someone to lookup an interpolated value by passing
 * in an input of type double into the getValue method.
 * 
 * @author Dhruv Shah
 */
public class TreeLookup {
    private TreeMap<Double, Double> treeMap;

    /**
     * Takes in a TreeMap of generic &lt;Double, Double&gt;
     * 
     * @param treeMap
     */
    public TreeLookup(TreeMap<Double, Double> treeMap) {
        this.treeMap = treeMap;
    }

    /**
     * Takes an input and returns an interpolated value
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

        return belowValue + (input - belowKey) * slope; // interpolation = y1 + (x-x1) * slope
    }

    /**
     * Takes an input and returns an interpolated value
     * 
     * @param input
     * @return interpolation value
     */
    static double getValue(TreeMap<Double, Double> treeMap, double input) {
        if (treeMap.containsKey(input)) return treeMap.get(input);

        double belowKey = treeMap.floorKey(input); // x1
        double belowValue = treeMap.get(belowKey); // y1
        double aboveKey = treeMap.ceilingKey(input); // x2
        double aboveValue = treeMap.get(aboveKey); // y2

        double slope = (aboveValue - belowValue) / (aboveKey - belowKey); // slope = (y2-y1) / (x2-x1)

        return belowValue + (input - belowKey) * slope; // interpolation = y1 + (x-x1) * slope
    }
}