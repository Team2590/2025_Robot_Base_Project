package treelookup;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.TreeLookup;
import java.util.TreeMap;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TreeLookupTest {
  static TreeMap<Double, Double> testMap = new TreeMap<Double, Double>();
  TreeLookup treeLookup = new TreeLookup(testMap);

  @BeforeAll
  static void setup() {
    testMap.put(2.0, 2.0);
    testMap.put(4.0, 4.0);
    testMap.put(6.0, 6.0);
  }

  @Test
  void testInterpolation() {
    double result = treeLookup.getValue(3);
    assertEquals(3, result);
  }

  @Test
  void testMinClamp() {
    double result = treeLookup.getValue(1);
    assertEquals(2, result);
  }

  @Test
  void testMaxClamp() {
    double result = treeLookup.getValue(5);
    assertEquals(5, result);
  }
}
