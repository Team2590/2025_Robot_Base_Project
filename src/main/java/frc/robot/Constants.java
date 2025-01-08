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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class CoralPose {
    public final double x;
    public final double y;
    public final double elevatorPosition;
    public final double rotationTarget;

    public CoralPose(double x, double y, double elevatorPosition, double rotationTarget) {
      this.x = x;
      this.y = y;
      this.elevatorPosition = elevatorPosition;
      this.rotationTarget = rotationTarget;
    }

    public CoralPose(double x, double y) {
      this.x = x;
      this.y = y;
      this.elevatorPosition = 0;
      this.rotationTarget = 0;
    }
  }

  public static class BlueCoralPoses {
    public static final CoralPose p1 = new CoralPose(3.9, 5.3);
    public static final CoralPose p2 = new CoralPose(3.676, 5);
    public static final CoralPose p3 = new CoralPose(3.14, 4.2);
    public static final CoralPose p4 = new CoralPose(3.15, 3.87);
    public static final CoralPose p5 = new CoralPose(3.62, 2.933);
    public static final CoralPose p6 = new CoralPose(3.93, 2.787);
    public static final CoralPose p7 = new CoralPose(5.031,2.787);
    public static final CoralPose p8 = new CoralPose(5.304,2.933);
    public static final CoralPose p9 = new CoralPose(5.83, 3.859);
    public static final CoralPose p10 = new CoralPose(5.83, 4.17);
    public static final CoralPose p11 = new CoralPose(5.275, 5.127);
    public static final CoralPose p12 = new CoralPose(3.968, 5.283);
  }

  public static class RedCoralPoses {
    public static final CoralPose p1 = new CoralPose(13.913, 5.088);
    public static final CoralPose p2 = new CoralPose(13.621, 5.253);
    public static final CoralPose p3 = new CoralPose(12.47, 5.253);
    public static final CoralPose p4 = new CoralPose(12.217, 5.127);
    public static final CoralPose p5 = new CoralPose(11.72, 4.2);
    public static final CoralPose p6 = new CoralPose(11.7, 3.869);
    public static final CoralPose p7 = new CoralPose(12.236, 2.943);
    public static final CoralPose p8 = new CoralPose(12.5, 2.806);
    public static final CoralPose p9 = new CoralPose(13.621, 2.777);
    public static final CoralPose p10 = new CoralPose(13.923, 2.933);
    public static final CoralPose p11 = new CoralPose(14.39, 3.869);
    public static final CoralPose p12 = new CoralPose(14.411, 4.171);
  }
}
