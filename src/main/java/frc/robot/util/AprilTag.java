package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTag {

  public static Pose2d[] TagPoses = {
    null,
    new Pose2d(16.70, 0.66, new Rotation2d(Math.toRadians(126))),
    new Pose2d(16.70, 7.40, new Rotation2d(Math.toRadians(-126))),
    new Pose2d(11.56, 8.06, new Rotation2d(Math.toRadians(-90))),
    new Pose2d(09.28, 6.14, new Rotation2d(Math.toRadians(0))),
    new Pose2d(09.28, 1.91, new Rotation2d(Math.toRadians(0))),
    new Pose2d(13.47, 3.31, new Rotation2d(Math.toRadians(-60))),
    new Pose2d(13.89, 4.03, new Rotation2d(Math.toRadians(0))),
    new Pose2d(13.47, 4.75, new Rotation2d(Math.toRadians(60))),
    new Pose2d(12.64, 4.75, new Rotation2d(Math.toRadians(120))),
    new Pose2d(12.23, 4.03, new Rotation2d(Math.toRadians(180))),
    new Pose2d(12.64, 3.31, new Rotation2d(Math.toRadians(-120))),
    new Pose2d(00.85, 0.66, new Rotation2d(Math.toRadians(54))),
    new Pose2d(00.85, 7.40, new Rotation2d(Math.toRadians(-54))),
    new Pose2d(08.27, 6.14, new Rotation2d(Math.toRadians(180))),
    new Pose2d(08.27, 1.91, new Rotation2d(Math.toRadians(180))),
    new Pose2d(05.99, 0.00, new Rotation2d(Math.toRadians(90))),
    new Pose2d(04.07, 3.31, new Rotation2d(Math.toRadians(-120))),
    new Pose2d(03.66, 4.03, new Rotation2d(Math.toRadians(180))),
    new Pose2d(04.07, 4.75, new Rotation2d(Math.toRadians(120))),
    new Pose2d(04.90, 4.75, new Rotation2d(Math.toRadians(60))),
    new Pose2d(05.32, 4.03, new Rotation2d(Math.toRadians(0))),
    new Pose2d(04.90, 3.31, new Rotation2d(Math.toRadians(-60)))
  };
}
