package treelookup;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.TreeMap;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.util.TreeLookup;

public class TreeLookupTest {
    private TreeMap<Double, Double> testMap = new TreeMap<Double, Double>();
    private TreeLookup treeLookup = new TreeLookup(null);

    @BeforeAll 
    void init() {
        testMap.put(2.0, 2.0);
        testMap.put(4.0, 4.0);
    }

    @Test
    public void testInterpolation() {
        double result = treeLookup.getValue(3);
        assertEquals(result, 3);
    }

    @Test
    public void testMinClamp() {
        double result = treeLookup.getValue(1);
        assertEquals(result, 2);
    }

    @Test
    public void testMaxClamp() {
        double result = treeLookup.getValue(5);
        assertEquals(result, 4);
    }
}