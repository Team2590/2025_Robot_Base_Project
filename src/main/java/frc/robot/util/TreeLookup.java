package frc.robot.util;

import java.util.TreeMap;

/**
 * A class to allow looking up values in a TreeMap
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
    if (treeMap.isEmpty()) throw new Error("The treemap is empty");
    if (treeMap.entrySet().size() < 2) throw new Error("The treemap has less than 2 entries");
    this.treeMap = treeMap;
  }

  /**
   * Takes an input key and returns an interpolated value
   *
   * @param input
   * @return interpolation value
   */
  public double getValue(double input) {
    if (input < treeMap.firstKey()) return treeMap.firstEntry().getValue();
    if (input > treeMap.lastKey()) return treeMap.lastEntry().getValue();
    var belowEntry = treeMap.floorEntry(input); 
    var aboveEntry = treeMap.ceilingEntry(input); 

    double belowKey = belowEntry.getKey(); // x1
    double aboveKey = aboveEntry.getKey(); // x2

    double belowValue = belowEntry.getValue(); // y1
    double aboveValue = aboveEntry.getValue(); // y2

    double slope = (aboveValue - belowValue) / (aboveKey - belowKey); // slope = (y2-y1) / (x2-x1)
    double interpolated =
        belowValue + (input - belowKey) * slope; // interpolation = y1 + (x-x1) * slope

    return interpolated;
  }
}
