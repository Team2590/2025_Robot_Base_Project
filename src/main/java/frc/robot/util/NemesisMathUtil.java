package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class NemesisMathUtil {
  public static boolean isApprox(Number value, Number tolerance, Number desired) {
    return value.doubleValue() > desired.doubleValue() - tolerance.doubleValue()
        && value.doubleValue() < desired.doubleValue() + tolerance.doubleValue();
  }

  public static boolean isBetweenInclusive(Number value, Number min, Number max) {
    return value.doubleValue() >= min.doubleValue() && value.doubleValue() <= max.doubleValue();
  }

  public static boolean isBetweenExclusive(Number value, Number min, Number max) {
    return value.doubleValue() > min.doubleValue() && value.doubleValue() < max.doubleValue();
  }

  public static boolean isTranslationApprox(
      Translation2d translation1, Translation2d translation2, Number tolerance) {
    return isApprox(translation1.getX(), translation2.getX(), tolerance)
        && isApprox(translation1.getY(), translation2.getY(), tolerance);
  }
}
