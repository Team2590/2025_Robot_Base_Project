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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FRCPolygon;
import frc.robot.util.PolygonLocator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

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

 public static final String REEF_BLUE_POLYGON = "reefBluePolygon";
 
 public static final FRCPolygon reefBluePolygon = 
      new FRCPolygon(
        REEF_BLUE_POLYGON,
        new Translation2d(3.389, 3.324),
        new Translation2d(4.497, 2.655),
        new Translation2d(5.690, 3.324),
        new Translation2d(5.690, 4.705),
        new Translation2d(4.497, 5.332),
        new Translation2d(3.389, 4.705));
  
  public static final String REEF_RED_POLYGON = "reefRedPolygon";
 
  public static final FRCPolygon reefRedPolygon = 
      new FRCPolygon(
        REEF_RED_POLYGON,
        new Translation2d(14.224, 4.663),
        new Translation2d(13.095, 5.332),
        new Translation2d(11.902, 4.663),
        new Translation2d(11.902, 3.303),
        new Translation2d(13.095, 2.655),
        new Translation2d(14.224, 3.303));

  public static final String BARGE_BLUE_POLYGON = "bargeBluePolygon";
 
  public static final FRCPolygon bargeBluePolygon = 
      new FRCPolygon(
        BARGE_BLUE_POLYGON,
        new Translation2d(8.221, 4.370),
        new Translation2d(9.371, 4.370),
        new Translation2d(9.371, 7.968),
        new Translation2d(8.221, 7.968));
  
  public static final String BARGE_RED_POLYGON = "bargeRedPolygon";
 
  public static final FRCPolygon bargeRedPolygon = 
      new FRCPolygon(
        BARGE_RED_POLYGON,
        new Translation2d(9.371, 3.680),
        new Translation2d(8.221, 3.680),
        new Translation2d(8.221, 0.032),
        new Translation2d(9.371, 0.032));
  // Two ways to instantiate the polygons, this static initialization box is necessary
  static {
    polygons.add(reefBluePolygon);
    polygons.add(reefRedPolygon);
    polygons.add(bargeBluePolygon);
    polygons.add(bargeRedPolygon);
  }

  public static PolygonLocator locator = new PolygonLocator(polygons, fieldBounds);

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
