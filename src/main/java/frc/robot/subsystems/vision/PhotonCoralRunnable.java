package frc.robot.subsystems.vision;

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
  private static double coralXOffset = 0;
  private static double coralYOffset = 0;

  public PhotonCoralRunnable() {
    CoralCam.setPipelineIndex(1);
  }

  /** Updates results on the note detection camera. */
  @Override
  public void run() {
    for (var result : CoralCam.getAllUnreadResults()) {
        if (result.hasTargets()) {
            target = result.getBestTarget();
            coralXOffset = camHeight / (Math.tan(camPitch - Math.toRadians(target.getPitch())));
            coralYOffset = coralXOffset * Math.tan(Math.toRadians(target.getYaw())) + camYOffset;
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
        return target.getYaw();
      } else {
        return 0;
      }
    } catch (NullPointerException e) {
      e.printStackTrace();
      return 0;
    }
  }

  /**
   * Gets the pitch of target in DEGREES.
   *
   * @return pitch of target in degrees
   */
  public static double getPitch() {
    try {
      if (result.hasTargets()) {
        return target.getPitch();
      } else {
        return 0;
      }
    } catch (NullPointerException e) {
      e.printStackTrace();
      return 0;
    }
  }
}