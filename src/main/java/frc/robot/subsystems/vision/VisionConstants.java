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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public static final String PHOTON_TEST_CAMERA = "PhotonTestCamera";

  // Robot to camera transforms
  public static Transform3d robotToPhotonTestCamera =
      new Transform3d(
          Units.inchesToMeters(8.625),
          Units.inchesToMeters(11.625),
          Units.inchesToMeters(9.6324),
          new Rotation3d(0, Math.toRadians(-6.1598479), Math.toRadians(9.0569)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters // .02
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {0.015};

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final Set<Integer> FIDUCIAL_IDS_RED = Set.of(6, 7, 8, 9, 10, 11);

  public static final Set<Integer> FIDUCIAL_IDS_BLUE = Set.of(17, 18, 19, 20, 21, 22);

  public static double DISTANCE_THRESHOLD = Units.inchesToMeters(120); // TODO: TUNE VALUE FOR COMP

  public static Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );
}
