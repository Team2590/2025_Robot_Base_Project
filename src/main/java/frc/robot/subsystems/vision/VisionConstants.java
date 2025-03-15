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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.Set;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // static {
  //   try {
  //     aprilTagLayout = new AprilTagFieldLayout("../../generated/commonsFieldCal3-11-2025.json");
  //   } catch (IOException e) {
  //     aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  //   }
  // }

  // Camera names, must match names configured on coprocessor
  public static String upperSourceCameraName = "1mp_arducam_device_6";
  //   public static String processorCameraName = "1mp_arducam_device_4";
  public static String reefCameraName = "1mp_arducam_device_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // Upper Source Camera
  public static Transform3d robotToUpperSourceCam =
      new Transform3d(
          Units.inchesToMeters(-7.8508),
          Units.inchesToMeters(9.0),
          Units.inchesToMeters(35.7347),
          new Rotation3d(0.0, Math.toRadians(-51.0), Math.toRadians(180.0)));

  // Processor Camera
  //   public static Transform3d robotToProcessorCam =
  //       new Transform3d(
  //           Units.inchesToMeters(-6.379),
  //           Units.inchesToMeters(7.2767),
  //           Units.inchesToMeters(29.9459),
  //           new Rotation3d(0.0, Math.toRadians(-27.0), Math.toRadians(-90.0)));

  // Reef Camera
  public static Transform3d robotToReefCam =
      new Transform3d(
          Units.inchesToMeters(5.0),
          Units.inchesToMeters(12.0),
          Units.inchesToMeters(10.33832),
          new Rotation3d(0.0, 0.0, Math.toRadians(-22.25)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters // .02
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.25, // Upper Source Camera
        // 0.25, // Processor Camera
        0.25 // Reef Camera
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final Set<Integer> FIDUCIAL_IDS =
      Set.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11);

  public static double DISTANCE_THRESHOLD = Units.inchesToMeters(120); // TODO: TUNE VALUE FOR COMP
}
