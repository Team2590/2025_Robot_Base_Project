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

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String sourceCameraName = "camera_0";
  public static String processorCameraName = "camera_1";
  public static String reefCameraName = "idk";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // Front side camera (camera0)
  public static Transform3d robotToSourceCam =
      new Transform3d(
          -0.2079,
          0.2269,
          0.9278,
          new Rotation3d(0.0, Math.toRadians(-51.0), Math.toRadians(180.0)));

  // Back side camera (camera1)
  public static Transform3d robotToProcessorCam =
      new Transform3d(
          -0.1869,
          0.2064,
          0.7892,
          new Rotation3d(0.0, Math.toRadians(-27.0), Math.toRadians(90.0)));

  // Left side camera (camera2)
  //   public static Transform3d robotToReefCam =
  //       new Transform3d(0.0, 0.2, 0.2, new Rotation3d(0.0, -0.4, Math.PI / 2));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // ,  Camera 1
        // 1.0, // Camera 2
        // 1.0 //  Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
