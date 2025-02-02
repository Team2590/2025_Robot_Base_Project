package frc.robot.util;

public class NemesisMathUtil {
  public static boolean isApprox(Number value, Number tolerance, Number desired) {
    return value.doubleValue() > desired.doubleValue() - tolerance.doubleValue()
        && value.doubleValue() < desired.doubleValue() + tolerance.doubleValue();
  }
}
