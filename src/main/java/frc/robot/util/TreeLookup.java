package frc.robot.util;

import java.util.TreeMap;

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
    this.min = treeMap.values().stream().min(Double::compareTo).orElse(Double.NaN);
    this.max = treeMap.values().stream().max(Double::compareTo).orElse(Double.NaN);
  }

  /**
   * Takes an input key and returns an interpolated value
   *
   * @param input
   * @return interpolation value
   */
  public double getValue(double input) {
    if (input > max) return max;
    if (input < min) return min;
    Double belowKey = treeMap.floorKey(input); // x1
    Double aboveKey = treeMap.ceilingKey(input); // x2

    double belowValue = treeMap.get(belowKey); // y1
    double aboveValue = treeMap.get(aboveKey); // y2

    double slope = (aboveValue - belowValue) / (aboveKey - belowKey); // slope = (y2-y1) / (x2-x1)
    double interpolated = belowValue + (input - belowKey) * slope; // interpolation = y1 + (x-x1) * slope

    return interpolated;
  }
}
