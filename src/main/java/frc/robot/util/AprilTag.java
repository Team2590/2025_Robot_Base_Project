package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTag {

  // April Tag positions on the field
  public static Pose3d[] TagPoses = {
    null,
    new Pose3d(
        16.70,
        0.66,
        1.49,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(126))),
    new Pose3d(
        16.70,
        7.40,
        1.49,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-126))),
    new Pose3d(
        11.56,
        8.06,
        1.30,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-90))),
    new Pose3d(
        09.28,
        6.14,
        1.87,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(0))),
    new Pose3d(
        09.28,
        1.91,
        1.87,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(0))),
    new Pose3d(
        13.47,
        3.31,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-60))),
    new Pose3d(
        13.89, 4.03, 0.31, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))),
    new Pose3d(
        13.47,
        4.75,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(60))),
    new Pose3d(
        12.64,
        4.75,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(120))),
    new Pose3d(
        12.23,
        4.03,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))),
    new Pose3d(
        12.64,
        3.31,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-120))),
    new Pose3d(
        00.85,
        0.66,
        1.49,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(54))),
    new Pose3d(
        00.85,
        7.40,
        1.49,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-54))),
    new Pose3d(
        08.27,
        6.14,
        1.87,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(180))),
    new Pose3d(
        08.27,
        1.91,
        1.87,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(180))),
    new Pose3d(
        05.99,
        0.00,
        1.30,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(90))),
    new Pose3d(
        04.07,
        3.31,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-120))),
    new Pose3d(
        03.66,
        4.03,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))),
    new Pose3d(
        04.07,
        4.75,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(120))),
    new Pose3d(
        04.90,
        4.75,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(60))),
    new Pose3d(
        05.32, 4.03, 0.31, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))),
    new Pose3d(
        04.90,
        3.31,
        0.31,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-60)))
  };
}
