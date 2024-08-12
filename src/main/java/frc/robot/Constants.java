// Copyright 2021-2024 FRC 6328
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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
  }

  public final class VisionConstants {
    // 1 of the april tag cameras
    public static final double CAMERA_1_HEIGHT_METERS = Units.inchesToMeters(18.75);
    public static final double CAMERA_1_X_DISTANCE_FROM_CENTER_METERS =
        Units.inchesToMeters(10.948);
    public static final double CAMERA_1_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(0);
    public static final double CAMERA_1_ROLL = 0;
    // downward pitch is positive
    public static final double CAMERA_1_PITCH = Units.degreesToRadians(-1 * 27);
    // counterclockwise yaw is positive
    public static final double CAMERA_1_YAW = Units.degreesToRadians(-1 * 180);
    public static final Transform3d RobotToCam_1 =
        new Transform3d(
            -CAMERA_1_X_DISTANCE_FROM_CENTER_METERS,
            CAMERA_1_Y_DISTANCE_FROM_CENTER_METERS,
            CAMERA_1_HEIGHT_METERS,
            new Rotation3d(CAMERA_1_ROLL, CAMERA_1_PITCH, CAMERA_1_YAW));

    // the other april tag camera
    public static final double CAMERA_2_HEIGHT_METERS = Units.inchesToMeters(18.75);
    public static final double CAMERA_2_X_DISTANCE_FROM_CENTER_METERS =
        Units.inchesToMeters(10.948);
    public static final double CAMERA_2_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(0);
    public static final double CAMERA_2_ROLL = 0;
    // downward pitch is positive
    public static final double CAMERA_2_PITCH = Units.degreesToRadians(-1 * 27);
    // counterclockwise yaw is positive
    public static final double CAMERA_2_YAW = Units.degreesToRadians(-1 * 180);
    public static final Transform3d RobotToCam_2 =
        new Transform3d(
            -CAMERA_2_X_DISTANCE_FROM_CENTER_METERS,
            CAMERA_2_Y_DISTANCE_FROM_CENTER_METERS,
            CAMERA_2_HEIGHT_METERS,
            new Rotation3d(CAMERA_2_ROLL, CAMERA_2_PITCH, CAMERA_2_YAW));

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }
}
