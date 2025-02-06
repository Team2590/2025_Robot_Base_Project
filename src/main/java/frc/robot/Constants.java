// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FRCPolygon;
import frc.robot.util.PolygonLocator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.util.GeometryUtil;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.KRONOS : simMode;
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;

  public static class DriveToPoseConstraints {
    public static double maxVelocityMPS = 0;
    public static double maxAccelerationMPSSq = 0;
    public static double maxAngularVelocityRadPerSec = 0;
    public static double maxAngularAccelerationRadPerSecSq = 0;

    public static PathConstraints pathConstraints =
        new PathConstraints(
            maxVelocityMPS,
            maxAccelerationMPSSq,
            maxAngularVelocityRadPerSec,
            maxAngularAccelerationRadPerSecSq);
  }

  private static List<FRCPolygon> polygons = new ArrayList<>();
  private static Rectangle2D fieldBounds = new Rectangle2D.Double(0, 0, 15, 15);
  /*
 * 
 * The ordering of points matters. I went mad trying to debug at one point, turns out it was in a bowtie and not a square
 * Bowtie shape (incorrect rectangle):
 * 
 *     (0,2)---(2,2)
 *      / \   / \
 *     /   \ /   \
 *    (0,0)---(2,0)
 * 
 * Square shape (correct rectangle):
 * 
 *    (0,2)---(2,2)
 *     |       |
 *     |       |
 *    (0,0)---(2,0) 
 */

  //public static final Pose2d origin = new Pose2d();

  // ReefSide1 is the closest to the driver's station and is numbered clockwise from there
  public static final Pose2d ReefSide1 = new Pose2d(new Translation2d(2.662,4.054), Rotation2d.fromDegrees(0));
  public static final Pose2d ReefSide2 = new Pose2d(new Translation2d(3.5,5.585), Rotation2d.fromDegrees(302.125));
  public static final Pose2d ReefSide3 = new Pose2d(new Translation2d(5.489,5.595), Rotation2d.fromDegrees(237.089));
  public static final Pose2d ReefSide4 = new Pose2d(new Translation2d(6.397, 3.950), Rotation2d.fromDegrees(180));
  public static final Pose2d ReefSide5 = new Pose2d(new Translation2d(5.438, 2.415), Rotation2d.fromDegrees(122.005));
  public static final Pose2d ReefSide6 = new Pose2d(new Translation2d(3.520, 2.426), Rotation2d.fromDegrees(59.194));

  public static final Pose2d Barge = new Pose2d(new Translation2d(7.576,6.150), Rotation2d.fromDegrees(0));

  public static final Pose2d CageShallow = new Pose2d(new Translation2d(8.023,6.146), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepLeft = new Pose2d(new Translation2d(8.023, 7.253), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepRight = new Pose2d(new Translation2d(8.023,5.059), Rotation2d.fromDegrees(0));

  public static final Pose2d CoralStationRight = new Pose2d(new Translation2d(1.567,6.579), Rotation2d.fromDegrees(125.538));

  public static final Pose2d CoralStationLeft = new Pose2d(new Translation2d(1.567,1.424), Rotation2d.fromDegrees(231.639));

  public static final Pose2d Processor = new Pose2d(new Translation2d(10.760,7.023), Rotation2d.fromDegrees(90));


  public static enum Mode {
    /** Running on a real robot. */
    COMP,

    KRONOS,

    LEO,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
