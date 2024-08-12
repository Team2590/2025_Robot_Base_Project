package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotContainer;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();
  public final ArrayList<TimestampedVisionUpdate> updates =
      new ArrayList<TimestampedVisionUpdate>();
  private static Pose3d RobotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static PhotonPipelineResult photonResults;
  private Transform3d cameraTransform;
  public final AprilTagFieldLayout layout;

  public PhotonRunnable(String name, Transform3d cameraTransform3d) {
    this.photonCamera = new PhotonCamera(name);
    this.cameraTransform = cameraTransform3d;
    PhotonPoseEstimator photonPoseEstimator = null;
    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // CHANGE TO 2025 GAME
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              photonCamera,
              this.cameraTransform);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();
      if (photonResults.hasTargets()) {
        Logger.recordOutput(
            "photonvision/FrontCamera/Tags In Sight", photonResults.targets.toString());
        if (isMultiDetection()
            || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD) {
          photonPoseEstimator
              .update(photonResults)
              .ifPresent(
                  estimatedRobotPose -> {
                    RobotPose = estimatedRobotPose.estimatedPose;
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    // Make sure the measurement is on the field
                    if (estimatedPose.getX() > 0.0
                        && estimatedPose.getX() <= FIELD_LENGTH_METERS
                        && estimatedPose.getY() > 0.0
                        && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                      atomicEstimatedRobotPose.set(estimatedRobotPose);
                      updates.add(getPoseAtTimestamp(timestamp));
                      RobotContainer.poseEstimator.addVisionData(updates);
                      updates.clear();
                    }
                  });
        }
      }
    }
  }

  public Pose3d getRobotPose3d() {
    return RobotPose;
  }

  public Pose2d getRobotPose2d() {
    return RobotPose.toPose2d();
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a
   * non-null value, it is a new estimate that hasn't been returned before. This pose will always be
   * for the BLUE alliance. It must be flipped if the current alliance is RED.
   *
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

  public TimestampedVisionUpdate getPoseAtTimestamp(double timestamp) {
    if(isMultiDetection()){
      return new TimestampedVisionUpdate(
        timestamp,
        grabLatestEstimatedPose().estimatedPose.toPose2d(),
        VecBuilder.fill(.001, .003, .005)); // SUBJECT TO CHANGE (multi tag constants)
    }
    double distance = getDistanceToBestTag();
    double n1 = .001 * distance; // SUBJECT TO CHANGE
    double n2 = .003 * distance; // SUBJECT TO CHANGE
    double n3 = .005 * distance; // SUBJECT TO CHANGE
    return new TimestampedVisionUpdate(
        timestamp,
        grabLatestEstimatedPose().estimatedPose.toPose2d(),
        VecBuilder.fill(n1, n2, n3));
  }

  // returns if multi tag detection is being used
  public boolean isMultiDetection() {
    return photonResults.targets.size() > 1;
  }

  // returns the distance in meters to the best target (either by distance or by confidence--can be
  // changed)
  public double getDistanceToBestTag() {
    PhotonTrackedTarget target = photonResults.getBestTarget();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    return Math.hypot(
        bestCameraToTarget.getZ(),
        Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY()));
  }
}
