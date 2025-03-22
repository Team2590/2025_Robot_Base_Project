package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralIOPhotonVision implements CoralDetectionIO {
  private final CoralDetectionThread thread;
  private final Object observationLock = new Object();

  public CoralIOPhotonVision() {
    thread = new CoralDetectionThread();
    thread.start();
  }

  @Override
  public void updateInputs(CoralDetectionIOInputs inputs) {
    synchronized (observationLock) {
      inputs.coralPose = thread.getCoralPose();
      inputs.coralRotation = thread.getCoralRotation();
      inputs.coralYaw = thread.getCoralYaw();
    }
  }

  class CoralDetectionThread extends Thread {
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private PhotonCamera CoralCam;
    public PhotonTrackedTarget target;
    private double camHeight =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_HEIGHT_METERS;
    private double camPitch = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_PITCH;
    private double camXOffset =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
    private double camYOffset =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
    private double camFocalLength =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_FOCAL_LENGTH;
    private double coralXOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_X_OFFSET;
    private double coralYOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_Y_OFFSET;
    private double coralYaw = 0;
    private Pose2d coralPose = RobotContainer.getDrive().getPose();
    private Transform2d robotToCoral = new Transform2d();
    private Rotation2d coralRotation = new Rotation2d();
    private volatile boolean connected = false;

    public CoralDetectionThread() {
      CoralCam.setPipelineIndex(VisionConstants.CoralAlgaeCameraConstants.CORAL_PIPELINE_INDEX);
      this.CoralCam =
          new PhotonCamera(instance, VisionConstants.CoralAlgaeCameraConstants.CAMERA_NAME);
      setDaemon(true);
    }

    public boolean isConnected() {
      return connected;
    }

    public Pose2d getCoralPose() {
      return coralPose;
    }

    public Rotation2d getCoralRotation() {
      return coralRotation;
    }

    public double getCoralYaw() {
      return coralYaw;
    }

    @Override
    public void run() {
      Pose2d robotPose = RobotContainer.getDrive().getPose();
      while (!Thread.interrupted()) {
        try {
          connected = CoralCam.isConnected();
          for (var result : CoralCam.getAllUnreadResults()) {
            synchronized (observationLock) {
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
                  double currentCoralYaw =
                      Math.toDegrees(Math.atan(yToCoral / (xToCoral + camXOffset)));
                  robotToCoral =
                      new Transform2d(coralXOffset, -coralYOffset - camYOffset, new Rotation2d());
                  coralPose = robotPose.plus(robotToCoral);

                  if (target != null) {
                    coralYaw = currentCoralYaw;
                    coralRotation = new Rotation2d(-currentCoralYaw);
                  } else {
                    coralYaw = 0;
                    coralRotation = new Rotation2d();
                  }
                } else {
                  target = null;
                  coralXOffset = 0;
                  coralYOffset = 0;
                  coralPose = robotPose;
                  coralYaw = 0;
                }
              } catch (NullPointerException e) {
                e.printStackTrace();
                coralPose = robotPose;
                coralYaw = 0;
                coralRotation = new Rotation2d();
              }
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
