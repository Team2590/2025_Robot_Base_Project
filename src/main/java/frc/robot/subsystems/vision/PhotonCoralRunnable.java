package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.vision.VisionConstants;

public class PhotonCoralRunnable implements Runnable {
  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static PhotonCamera CoralCam = new PhotonCamera(instance, "fisheye");
  private static PhotonPipelineResult result;
  public static PhotonTrackedTarget target;
  private static double camHeight = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_HEIGHT_METERS;
  private static double camPitch = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_PITCH;
  private static double camXOffset = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
  private static double camYOffset = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
  private static double camFocalLength = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_FOCAL_LENGTH;
  private static double coralXOffset = 0;
  private static double coralYOffset = 0;
  private static double coralYaw;
  private static Transform2d robotToCoral;

  public PhotonCoralRunnable() {
    CoralCam.setPipelineIndex(VisionConstants.CoralAlgaeCameraConstants.CORAL_PIPELINE_INDEX);
    robotToCoral = new Transform2d();
  }

  /** Updates results on the note detection camera. */
  @Override
  public void run() {
    for (var result : CoralCam.getAllUnreadResults()) {
      if (result.hasTargets()) {
        target = result.getBestTarget();
        double mPitch = Math.toRadians(target.getPitch());
        double mYaw = Math.toRadians(target.getYaw());
        double realPitch = Math.atan2(mPitch, camFocalLength);
        double realYaw = Math.atan2(mYaw, camFocalLength);
        double distanceToNote = camHeight / (Math.tan(camPitch - realPitch));
        double xToNote = distanceToNote * Math.cos(realYaw);
        double yToNote = distanceToNote * Math.sin(realYaw);
        coralYaw = Math.toDegrees(Math.atan(yToNote / (xToNote + camXOffset)));
        robotToCoral = new Transform2d(coralXOffset, -coralYOffset - camYOffset, new Rotation2d());
      } else {
            target = null;
                coralXOffset = 0;
                coralYOffset = 0;
              }
    }
    
  }

  /**
   * Gets the HORIZONTAL (right is +) translation from the center of the robot to the detected note.
   *
   * @return y offset of note
   */
  public static double getYOffset() {
    return coralYOffset;
  }

  public static double getXOffset() {
    return coralXOffset + camXOffset;
  }

  /**
   * Gets the yaw of target in DEGREES.
   *
   * @return yaw of target in degrees
   */
  public static double getYaw() {
    try {
      if (target != null && result.hasTargets()) {
        return coralYaw;
      } else {
        return 0;
      }
    } catch (NullPointerException e) {
      e.printStackTrace();
      return 0;
    }
  }

  public static Pose2d getCoralPose(Pose2d robotPose){
    try {
      if (result.hasTargets()) {
        return robotPose.plus(robotToCoral);
      } else {
        return robotPose;
      }
    } catch (NullPointerException e) {
      e.printStackTrace();
      return robotPose;
    }
  }
}