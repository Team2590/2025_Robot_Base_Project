package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
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
    private double camYaw = VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_YAW;
    private double camXOffset =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
    private double camYOffset =
        VisionConstants.CoralAlgaeCameraConstants.OBJECT_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
    private LoggedTunableNumber camFocalLength =
        new LoggedTunableNumber("Vision/CoralCamFocalLength", 0.39);
    private double coralXOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_X_OFFSET;
    private double coralYOffset = VisionConstants.CoralAlgaeCameraConstants.CORAL_Y_OFFSET;
    private double coralYaw = 0;
    private Pose2d coralPose = RobotContainer.getDrive().getPose();
    private Rotation2d coralRotation = new Rotation2d();
    private volatile boolean connected = false;

    public CoralDetectionThread() {
      // CoralCam.setPipelineIndex(VisionConstants.CoralAlgaeCameraConstants.CORAL_PIPELINE_INDEX);
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
      return -coralYaw;
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
                  double realPitch = Math.atan2(mPitch, camFocalLength.getAsDouble());
                  double realYaw = Math.atan2(mYaw, camFocalLength.getAsDouble());
                  double x_z_mag = camHeight / Math.sin(-realPitch + camPitch);
                  double x_comp = x_z_mag * Math.cos(realPitch);
                  double y_comp = x_z_mag * Math.tan(realYaw);
                  double z_comp = x_comp * Math.sin(realPitch);
                  Transform3d cameraToCoral =
                      new Transform3d(x_comp, -y_comp, z_comp, new Rotation3d(0, 0, 0));
                  double currentCoralYaw =
                      Math.toDegrees(Math.atan((y_comp - camYOffset) / (x_comp - camXOffset)));
                  coralPose =
                      new Pose3d(
                              robotPose.getX(),
                              robotPose.getY(),
                              0,
                              new Rotation3d(robotPose.getRotation()))
                          .plus(
                              VisionConstants.CoralAlgaeCameraConstants.robotToObjectCamera.plus(
                                  cameraToCoral))
                          .toPose2d();

                  if (target != null) {
                    coralYaw = currentCoralYaw;
                    coralRotation = new Rotation2d(-currentCoralYaw);
                  }
                } else {
                  target = null;
                }
              } catch (NullPointerException e) {
                e.printStackTrace();
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
