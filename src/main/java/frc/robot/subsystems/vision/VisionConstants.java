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
  public static String frontTopReefCameraName = "1mp_arducam_device_6";
  public static String frontBottomReefCameraName = "1mp_arducam_device_3";
  public static String backTopReefCameraName = "1mp_arducam_device_1";
  public static String backBottomReefCameraName = "1mp_arducam_device_8";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  //   // Front Top Reef Camera
  //   public static Transform3d robotToFrontTopReefCam =
  //       new Transform3d(
  //           Units.inchesToMeters(7.5),
  //           Units.inchesToMeters(11.5),
  //           Units.inchesToMeters(9.5),
  //           new Rotation3d(Math.PI, Math.toRadians(-6.1598479), Math.toRadians(9.0569)));

  //   // Front Bottom Reef Camera
  //   public static Transform3d robotToFrontBottomReefCam =
  //       new Transform3d(
  //           Units.inchesToMeters(9.5),
  //           Units.inchesToMeters(13.0),
  //           Units.inchesToMeters(7.5),
  //           new Rotation3d(Math.PI, Math.toRadians(-19.382516), Math.toRadians(-45.943058)));

  // Front Top Reef Camera
  public static Transform3d robotToFrontTopReefCam =
      new Transform3d(
          Units.inchesToMeters(8.625),
          Units.inchesToMeters(11.625),
          Units.inchesToMeters(9.6324),
          new Rotation3d(Math.PI, Math.toRadians(-6.1598479), Math.toRadians(9.0569)));

  // Front Bottom Reef Camera
  public static Transform3d robotToFrontBottomReefCam =
      new Transform3d(
          Units.inchesToMeters(10.5164),
          Units.inchesToMeters(12.7448),
          Units.inchesToMeters(7.5324),
          new Rotation3d(Math.PI, Math.toRadians(-19.382516), Math.toRadians(-45.943058)));

  // Back Top Reef Camera
  public static Transform3d robotToBackTopReefCam =
      new Transform3d(
          Units.inchesToMeters(-8.625),
          Units.inchesToMeters(11.625),
          Units.inchesToMeters(9.6324),
          new Rotation3d(Math.PI, Math.toRadians(-6.1598479), Math.toRadians(180 - 9.0569)));

  // Back Bottom Reef Camera
  public static Transform3d robotToBackBottomReefCam =
      new Transform3d(
          Units.inchesToMeters(-10.5164),
          Units.inchesToMeters(12.7448),
          Units.inchesToMeters(7.5324),
          new Rotation3d(Math.PI, Math.toRadians(-19.382516), Math.toRadians(180 + 45.943058)));

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
        0.25, // Front Top Reef Camera
        0.25 // Front Bottom Reef Camera
        // 0.25, // Back Top Reef Camera
        // 0.25 // Front Bottom Reef Camera
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor = .015; // No rotation data available

  public static final Set<Integer> FIDUCIAL_IDS_RED = Set.of(6, 7, 8, 9, 10, 11);

  public static final Set<Integer> FIDUCIAL_IDS_BLUE = Set.of(17, 18, 19, 20, 21, 22);

  public class CoralAlgaeCameraConstants {
    public static double OBJECT_CAMERA_HEIGHT_METERS =
        0.435 - 0.115
            - 0.035; // -0.115 for height of coral & -0.035 bc extra gap when detecting coral
    public static double OBJECT_CAMERA_PITCH = Math.toRadians(38);
    public static double OBJECT_CAMERA_YAW = Math.toRadians(45);
    public static double OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(0);
    public static double OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS = 0;

    public static Transform3d robotToObjectCamera =
        new Transform3d(
            OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS,
            OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS,
            OBJECT_CAMERA_HEIGHT_METERS,
            new Rotation3d(
                0, // ROLL
                OBJECT_CAMERA_PITCH, // PITCH
                OBJECT_CAMERA_YAW // YAW
                ));

    public static int CORAL_PIPELINE_INDEX = 0;
    public static String CAMERA_NAME = "Arducam_OV9782_USB_Camera";
    public static double CORAL_X_OFFSET = 0;
    public static double CORAL_Y_OFFSET = 0;
  }

  public static double DISTANCE_THRESHOLD = Units.inchesToMeters(1200); // TODO: TUNE VALUE FOR COMP
}
