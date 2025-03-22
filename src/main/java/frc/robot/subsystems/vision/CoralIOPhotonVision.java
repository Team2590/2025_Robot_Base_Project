package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralIOPhotonVision implements CoralDetectionIO {
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private PhotonCamera CoralCam;
  public PhotonTrackedTarget target;
  private double camHeight = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_HEIGHT_METERS;
  private double camPitch = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_PITCH;
  private double camXOffset =
      VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
  private double camYOffset =
      VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
  private double camFocalLength =
      VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_FOCAL_LENGTH;
  private double coralXOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_X_OFFSET;
  private double coralYOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_Y_OFFSET;
  private double coralYaw;
  private Transform2d robotToCoral;

  public CoralIOPhotonVision() {
    CoralCam.setPipelineIndex(VisionConstants.CoralAlgaeCameraConstants.CORAL_PIPELINE_INDEX);
    robotToCoral = new Transform2d();
    this.CoralCam =
        new PhotonCamera(instance, VisionConstants.CoralAlgaeCameraConstants.CAMERA_NAME);
  }

  @Override
  public void updateInputs(CoralDetectionIOInputs inputs) {
    Pose2d robotPose = RobotContainer.getDrive().getPose();

    for (var result : CoralCam.getAllUnreadResults()) {
      try {
        if (result.hasTargets()) {
          target = result.getBestTarget();
          double mPitch = Math.toRadians(target.getPitch());
          double mYaw = Math.toRadians(target.getYaw());
          double realPitch = Math.atan2(mPitch, camFocalLength);
          double realYaw = Math.atan2(mYaw, camFocalLength);
          double distanceToCoral = camHeight / (Math.tan(camPitch - realPitch));
          double xToCoral = distanceToCoral * Math.cos(realYaw);
          double yToCoral = distanceToCoral * Math.sin(realYaw);
          inputs.coralYaw = Math.toDegrees(Math.atan(yToCoral / (xToCoral + camXOffset)));
          inputs.robotToCoral =
              new Transform2d(coralXOffset, -coralYOffset - camYOffset, new Rotation2d());
          inputs.coralPose = robotPose.plus(robotToCoral);

          if (target != null) {
            inputs.coralYaw = coralYaw;
            inputs.coralRotation = new Rotation2d(-coralYaw);
          } else {
            inputs.coralYaw = 0;
            inputs.coralRotation = new Rotation2d();
          }
        } else {
          target = null;
          coralXOffset = 0;
          coralYOffset = 0;
          inputs.coralPose = robotPose;
          inputs.coralYaw = 0;
        }
      } catch (NullPointerException e) {
        e.printStackTrace();
        inputs.coralPose = robotPose;
        inputs.coralYaw = 0;
        inputs.coralRotation = new Rotation2d();
      }
    }
  }
}
