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
import frc.robot.util.LoggedTunableNumber;
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
  public static final FRCPolygon playBox =
      new FRCPolygon(
          "playBox",
          new Translation2d(0, 0),
          new Translation2d(2, 0),
          new Translation2d(2, 2),
          new Translation2d(0, 2));

  // Two ways to instantiate the polygons, this static initialization box is necessary
  static {
    polygons.add(playBox);
    polygons.add(
        new FRCPolygon(
            "triangle",
            new Translation2d(5, 5),
            new Translation2d(6, 5),
            new Translation2d(5.5, 6)));
  }

  public static PolygonLocator locator = new PolygonLocator(polygons, fieldBounds);
  public static LoggedTunableNumber homeSetpoint =
      new LoggedTunableNumber("Arm/IntakeSetpoint", .155);
  public static final String CANBUS = "Takeover";

  public final class ArmConstants {
    // Fill in
    public static final double HOME_SETPOINT = homeSetpoint.get();
    public static final double CLIMB_SETPOINT = .198;
    public static final double INTAKE_SETPOINT = homeSetpoint.get();
    public static double REEF_1_SETPOINT = -0.27;
    public static double REEF_2_SETPOINT = -0.32;
    public static final int ARM = 45;
    public static final int ARM_CANCODER_ID = 44;
    public static final double ARM_GEAR_RATIO = 266.67;
    public static final double MAG_OFFSET = -.156;
    public static final double ARM_MAX = -0.35; // -.3
  }

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
