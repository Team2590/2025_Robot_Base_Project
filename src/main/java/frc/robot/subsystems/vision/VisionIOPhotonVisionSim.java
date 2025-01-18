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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new VisionIOPhotonVisionSim with multiple cameras.
   *
   * @param cameraConfigs List of camera configurations
   * @param poseSupplier Supplier for the robot pose to use in simulation
   */
  public VisionIOPhotonVisionSim(
      List<CameraConfig> cameraConfigs, Supplier<Pose2d> poseSupplier) {
    super(cameraConfigs);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim cameras for each camera config
    var cameraProperties = new SimCameraProperties();
    for (CameraThread thread : cameraThreads) {
      PhotonCameraSim cameraSim = new PhotonCameraSim(thread.getCamera(), cameraProperties);
      visionSim.addCamera(cameraSim, thread.getRobotToCamera());
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
