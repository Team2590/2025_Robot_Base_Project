package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
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
    return isApprox(Math.abs(translation2.getDistance(translation1)), tolerance, 0);
  }

  public static boolean isPoseApprox(Pose2d pose1, Pose2d pose2, Number tolerance) {
    return isTranslationApprox(pose1.getTranslation(), pose2.getTranslation(), tolerance)
        && isApprox(pose1.getRotation().getRadians(), pose2.getRotation().getRadians(), tolerance);
  }

  public static double distance(Pose2d pose1, Pose2d pose2) {
    return Math.hypot(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY());
  }
}
