package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class GeometryUtil {
  public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return Math.hypot(pose2.getX() - pose1.getX(), pose2.getY() - pose1.getY());
  }

  public static double horizontalError(Pose2d targetPose, Pose2d robotPose) {
    if (targetPose == null || robotPose == null) {
      return 0;
    }
    return targetPose.minus(robotPose).getY();
  }

  public static double angleError(Pose2d targetPose, Pose2d robotPose) {
    if (targetPose == null || robotPose == null) {
      return 0;
    }
    return targetPose.minus(robotPose).getRotation().getRadians();
  }
}
