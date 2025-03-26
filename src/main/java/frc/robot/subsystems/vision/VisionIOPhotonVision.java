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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.RobotState.AligningState;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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
    private PhotonPoseEstimator photonPoseEstimator;
    private Pose3d robotReferencePose;

    public CameraThread(String name, Transform3d robotToCamera) {
      super("Vision-" + name);
      this.camera = new PhotonCamera(name);
      this.robotToCamera = robotToCamera;
      setDaemon(true);
      photonPoseEstimator =
          new PhotonPoseEstimator(
              VisionConstants.aprilTagLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              robotToCamera);
      robotReferencePose = new Pose3d();
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

    private void setEstimatedGlobalPose(
        Pose3d prevEstimatedRobotPose, PhotonPipelineResult result) {
      AligningState aligningState = RobotState.getInstance().getAligningState();
      boolean isAligning =
          aligningState == AligningState.ALIGNING_FRONT
              || aligningState == AligningState.ALIGNING_BACK;
      if (isAligning) {
        photonPoseEstimator.setPrimaryStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      } else {
        photonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      }
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);
      if (!estimatedRobotPose.isEmpty()) {
        robotReferencePose = estimatedRobotPose.get().estimatedPose;
        if (!isAligning && result.targets.size() > 1) {
          var multitagResult = result.getMultiTagResult().get();
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }

          latestTagIds.addAll(multitagResult.fiducialIDsUsed);

          latestPoseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotReferencePose,
                  multitagResult.estimatedPose.ambiguity,
                  multitagResult.fiducialIDsUsed.size(),
                  totalTagDistance / result.targets.size(),
                  PoseObservationType.PHOTONVISION));
        } else if (result.hasTargets()) {
          double tagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();

          latestTagIds.add((short) result.getBestTarget().getFiducialId());

          boolean logResults =
              (aligningState == AligningState.ALIGNING_FRONT
                      && (
                        this.getName().equals("Vision-" + VisionConstants.frontTopReefCameraName)
                        || this.getName().equals("Vision-" + VisionConstants.frontBottomReefCameraName)
                      ))
                  || (aligningState == AligningState.ALIGNING_BACK
                  && (
                    this.getName().equals("Vision-" + VisionConstants.backTopReefCameraName)
                    || this.getName().equals("Vision-" + VisionConstants.backBottomReefCameraName)
                  ))
                  || (aligningState == AligningState.NOT_ALIGNING);

          if (logResults) {
            latestPoseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(),
                    robotReferencePose,
                    result.getBestTarget().getPoseAmbiguity(),
                    1,
                    tagDistance,
                    PoseObservationType.PHOTONVISION));
          }
        }
      }
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
                for (int i = 0; i < result.targets.size(); i++) {
                  Transform3d camTransform3d = result.targets.get(i).getBestCameraToTarget();
                  double distance =
                      Math.hypot(
                          Math.hypot(camTransform3d.getX(), camTransform3d.getY()),
                          camTransform3d.getZ());
                  boolean isInWhiteList =
                      (DriverStation.getAlliance().isPresent()
                          && ((VisionConstants.FIDUCIAL_IDS_RED.contains(
                                      result.targets.get(i).getFiducialId())
                                  && DriverStation.getAlliance().get().equals(Alliance.Red))
                              || (VisionConstants.FIDUCIAL_IDS_BLUE.contains(
                                      result.targets.get(i).getFiducialId())
                                  && DriverStation.getAlliance().get().equals(Alliance.Blue))));
                  if ((distance >= VisionConstants.DISTANCE_THRESHOLD) || !isInWhiteList) {
                    // System.out.println(distance);
                    // System.out.println(targets.get(i));
                    result.targets.remove(i);
                    i--;
                  }
                }

                if (result.hasTargets()) {
                  latestTargetObservation =
                      new TargetObservation(
                          Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                          Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
                }
              }
              setEstimatedGlobalPose(robotReferencePose, result);
            }
          }
          sleep(20);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
  }
}
