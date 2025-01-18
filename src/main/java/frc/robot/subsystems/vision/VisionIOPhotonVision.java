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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  public static record CameraConfig(String name, Transform3d robotToCamera) {}

  protected final List<CameraThread> cameraThreads = new ArrayList<>();
  private final Object observationLock = new Object();
  private List<PoseObservation> latestPoseObservations = new ArrayList<>();
  private Set<Short> latestTagIds = new HashSet<>();
  private TargetObservation latestTargetObservation =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  /**
   * Creates a new VisionIOPhotonVision with multiple cameras.
   *
   * @param cameraConfigs List of camera configurations (name and transform)
   */
  public VisionIOPhotonVision(List<CameraConfig> cameraConfigs) {
    for (CameraConfig config : cameraConfigs) {
      CameraThread thread = new CameraThread(config.name(), config.robotToCamera());
      cameraThreads.add(thread);
      thread.start();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    synchronized (observationLock) {
      // Update connected status - true if any camera is connected
      inputs.connected = cameraThreads.stream().anyMatch(CameraThread::isConnected);

      // Copy latest observations to inputs
      inputs.poseObservations = latestPoseObservations.toArray(new PoseObservation[0]);
      inputs.tagIds = latestTagIds.stream().mapToInt(Integer::valueOf).toArray();
      inputs.latestTargetObservation = latestTargetObservation;

      // Clear for next update
      latestPoseObservations.clear();
      latestTagIds.clear();
    }
  }

  protected class CameraThread extends Thread {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private volatile boolean connected = false;

    public CameraThread(String name, Transform3d robotToCamera) {
      super("Vision-" + name);
      this.camera = new PhotonCamera(name);
      this.robotToCamera = robotToCamera;
      setDaemon(true);
    }

    public boolean isConnected() {
      return connected;
    }

    public PhotonCamera getCamera() {
      return camera;
    }

    public Transform3d getRobotToCamera() {
      return robotToCamera;
    }

    @Override
    public void run() {
      while (!Thread.interrupted()) {
        try {
          connected = camera.isConnected();

          // Process camera results
          for (var result : camera.getAllUnreadResults()) {
            synchronized (observationLock) {
              // Update target observation
              if (result.hasTargets()) {
                latestTargetObservation =
                    new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
              }

              // Process pose observations (existing multi-tag and single-tag logic)
              if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose =
                    new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                  totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                latestTagIds.addAll(multitagResult.fiducialIDsUsed);

                latestPoseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.targets.size(),
                        PoseObservationType.PHOTONVISION));
              } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                  Transform3d fieldToTarget =
                      new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                  Transform3d cameraToTarget = target.bestCameraToTarget;
                  Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                  Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                  Pose3d robotPose =
                      new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                  // Add tag ID
                  latestTagIds.add((short) target.fiducialId);

                  // Add observation
                  latestPoseObservations.add(
                      new PoseObservation(
                          result.getTimestampSeconds(), // Timestamp
                          robotPose, // 3D pose estimate
                          target.poseAmbiguity, // Ambiguity
                          1, // Tag count
                          cameraToTarget.getTranslation().getNorm(), // Average tag distance
                          PoseObservationType.PHOTONVISION)); // Observation type
                }
              }
            }
          }

          // Small sleep to prevent CPU overuse
          Thread.sleep(5);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
  }
}
