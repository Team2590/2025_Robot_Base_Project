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

import edu.wpi.first.apriltag.AprilTagFieldLayout; // Assuming this was implicitly available
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult; // Import needed for result type

/**
 * IO implementation for real PhotonVision hardware.
 * Processes camera data sequentially within updateInputs.
 */
public class VisionIOPhotonVision implements VisionIO {
    public static record CameraConfig(String name, Transform3d robotToCamera) {}

    // Store cameras and transforms directly
    private final List<PhotonCamera> cameras = new ArrayList<>();
    private final List<Transform3d> robotToCameraTransforms = new ArrayList<>();
    private final AprilTagFieldLayout aprilTagLayout = FieldConstants.aprilTagLayout; // Assuming needed and available (used in original thread)

    // Keep the original member variables for accumulating data within updateInputs
    private List<PoseObservation> latestPoseObservations = new ArrayList<>();
    private Set<Short> latestTagIds = new HashSet<>();
    private TargetObservation latestTargetObservation =
            new TargetObservation(new Rotation2d(), new Rotation2d());

    /**
     * Creates a new VisionIOPhotonVision with multiple cameras.
     *
     * @param cameraConfigs List of camera configurations (name and transform)
     * @param layout The AprilTagFieldLayout instance.
     */
    public VisionIOPhotonVision(List<CameraConfig> cameraConfigs) {
        for (CameraConfig config : cameraConfigs) {
            // Directly create and store cameras and transforms
            PhotonCamera camera = new PhotonCamera(config.name());
            cameras.add(camera);
            robotToCameraTransforms.add(config.robotToCamera());
            // Removed thread creation and starting
        }
        // Removed thread start loop and sleep
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Clear lists at the beginning of the update cycle
        latestPoseObservations.clear();
        latestTagIds.clear();
        // NOTE: latestTargetObservation is NOT cleared here, mimicking original logic
        // where it held the last value until overwritten by a new result.

        // Reset connected status, determined fresh each cycle
        inputs.connected = false;

        // Loop through each camera sequentially
        for (int i = 0; i < cameras.size(); i++) {
            PhotonCamera camera = cameras.get(i);
            Transform3d robotToCamera = robotToCameraTransforms.get(i);

            // Check connection status for this camera
            boolean currentCameraConnected = camera.isConnected();
            if (currentCameraConnected) {
                inputs.connected = true; // If any camera connects, mark as connected
            }

            // Process results if connected (or even if not? Original thread checked inside loop)
            // Replicating original logic: check connection, then process results.
            // A disconnected camera would just return empty results.
            try { // Add try-catch around result processing for robustness per camera
                for (var result : camera.getAllUnreadResults()) {
                    // Logic moved directly from CameraThread.run(), NO locking needed

                    // Update target observation (Original target filtering logic)
                    if (result.hasTargets()) {
                        // Original loop for filtering/removing targets
                        for (int targetIdx = 0; targetIdx < result.targets.size(); targetIdx++) {
                            Transform3d camTransform3d = result.targets.get(targetIdx).getBestCameraToTarget();
                            double distance =
                                    Math.hypot(
                                            Math.hypot(camTransform3d.getX(), camTransform3d.getY()),
                                            camTransform3d.getZ());
                            boolean isInWhiteList =
                                    (DriverStation.getAlliance().isPresent()
                                            && ((VisionConstants.FIDUCIAL_IDS_RED.contains(
                                                            result.targets.get(targetIdx).getFiducialId())
                                                    && DriverStation.getAlliance().get().equals(Alliance.Red))
                                                || (VisionConstants.FIDUCIAL_IDS_BLUE.contains(
                                                            result.targets.get(targetIdx).getFiducialId())
                                                    && DriverStation.getAlliance().get().equals(Alliance.Blue))));
                            if ((distance >= VisionConstants.DISTANCE_THRESHOLD) || !isInWhiteList) {
                                // System.out.println(distance);
                                // System.out.println(targets.get(i));
                                result.targets.remove(targetIdx);
                                targetIdx--; // Adjust index after removal
                            }
                            // System.out.println(result.targets);
                        }

                        // Update latestTargetObservation if targets remain *after* filtering
                        if (result.hasTargets()) {
                            latestTargetObservation =
                                    new TargetObservation(
                                            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                                            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
                        }
                    }

                    // Process pose observations (Original multi-tag and single-tag logic)
                    if (result.targets != null && result.targets.size() > 1) {
                        var multitagResult = result.multitagResult.get();
                        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                        Pose3d robotPose =
                                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                        double totalTagDistance = 0.0;
                        // Iterate using the filtered targets list
                        for (var target : result.targets) {
                            totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                        }
                        double avgTagDistance = multitagResult.fiducialIDsUsed.isEmpty() ? 0 : totalTagDistance / multitagResult.fiducialIDsUsed.size();


                        latestTagIds.addAll(multitagResult.fiducialIDsUsed);

                        latestPoseObservations.add(
                                new PoseObservation(
                                        result.getTimestampSeconds(),
                                        robotPose,
                                        multitagResult.estimatedPose.ambiguity,
                                        multitagResult.fiducialIDsUsed.size(), // Use size of IDs used
                                        avgTagDistance, // Use calculated average distance
                                        PoseObservationType.PHOTONVISION)); // Keep original type designation
                    } else if (!result.targets.isEmpty()) { // Only process single tags if multi-tag failed or wasn't possible
                        // Use the best target remaining after filtering
                        var target = result.targets.get(0); // Original logic used index 0

                        // Calculate robot pose (Original single-tag logic)
                        var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                        if (tagPose.isPresent()) {
                            Transform3d fieldToTarget =
                                    new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                            Transform3d cameraToTarget = target.getBestCameraToTarget();
                            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                            Pose3d robotPose =
                                    new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                            // Add tag ID
                            latestTagIds.add((short) target.getFiducialId());

                            // Add observation
                            latestPoseObservations.add(
                                    new PoseObservation(
                                            result.getTimestampSeconds(), // Timestamp
                                            robotPose, // 3D pose estimate
                                            target.getPoseAmbiguity(), // Ambiguity
                                            1, // Tag count
                                            cameraToTarget.getTranslation().getNorm(), // Average tag distance
                                            PoseObservationType.PHOTONVISION)); // Observation type
                        }
                    }
                    // End of processing for a single PhotonPipelineResult
                } // End loop over results from one camera
            } catch (Exception e) {
                // Log error for this camera, but continue processing others
                DriverStation.reportError("[VisionIOPhotonVision] Error processing results from camera: " + camera.getName() + " - " + e.getMessage(), e.getStackTrace());
            }
        } // End loop through all cameras

        // Copy accumulated data to inputs struct
        inputs.poseObservations = latestPoseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = latestTagIds.stream().mapToInt(Integer::valueOf).toArray();
        inputs.latestTargetObservation = latestTargetObservation;
        // Lists are cleared at the start of the next call
    }
}