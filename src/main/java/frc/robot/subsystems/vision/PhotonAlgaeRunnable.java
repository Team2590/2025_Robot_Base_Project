package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAlgaeRunnable implements Runnable {
  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static PhotonCamera AlgaeCam = new PhotonCamera(instance, "fisheye");
  private static PhotonPipelineResult result;
  public static PhotonTrackedTarget target;
  private static double camHeight = VisionConstants.OBJECT_CAMERA_HEIGHT_METERS;
  private static double camPitch = VisionConstants.OBJECT_CAMERA_PITCH;
  private static double camXOffset = VisionConstants.OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
  private static double camYOffset = VisionConstants.OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
  private static double algaeXOffset = 0;
  private static double algaeYOffset = 0;

  public PhotonAlgaeRunnable() {
    AlgaeCam.setPipelineIndex(0);
  }

  /** Updates results on the note detection camera. */
  @Override
  public void run() {
    try {
      result = AlgaeCam.getLatestResult();
      if (result.hasTargets()) {
        target = result.getBestTarget();
        algaeXOffset = camHeight / (Math.tan(camPitch - Math.toRadians(target.getPitch())));
        algaeYOffset = algaeXOffset * Math.tan(Math.toRadians(target.getYaw())) + camYOffset;
      } else {
        target = null;
        algaeXOffset = 0;
        algaeYOffset = 0;
      }
    } catch (NullPointerException e) {
      return;
    }
  }

  /**
   * Gets the HORIZONTAL (right is +) translation from the center of the robot to the detected note.
   *
   * @return y offset of note
   */
  public static double getYOffset() {
    return algaeYOffset;
  }

  public static double getXOffset() {
    return algaeXOffset + camXOffset;
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
