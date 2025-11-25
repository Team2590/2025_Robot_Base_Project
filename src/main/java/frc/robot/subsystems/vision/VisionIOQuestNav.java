package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav;
  private final String name;

  public VisionIOQuestNav(String name) {
    this.name = name;
    this.questNav = new QuestNav();
  }

  public VisionIOQuestNav(String name, QuestNav questNav) {
    this.name = name;
    this.questNav = questNav;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = questNav.isConnected();

    // Only update if connected
    if (inputs.connected) {
      List<PoseObservation> observations = new ArrayList<>();
      double timestamp = Timer.getFPGATimestamp() - questNav.getTimestamp(); // Approximate latency

      // QuestNav only provides robot pose directly, not camera pose.
      // We will assume a dummy camera pose and then convert it to robot pose later
      // The current Vision.java converts from camera to robot pose,
      // so we will provide robot pose as camera pose here and adjust Vision.java to handle it.
      // Alternatively, we could convert QuestNav's robot pose to a dummy camera pose using
      // robotToCamera transform, but that might be more complex.

      // For now, let's just pass the robot pose directly as the observation pose
      // and update the Vision subsystem to handle it.

      // A simple solution is to return the robot's pose directly as the observation.pose,
      // and ensure the Vision subsystem uses this directly as a robot pose, not a camera pose.
      // This means we might need to modify Vision.java to have a specific path for QuestNav
      // observations that are already in robot frame.

      // Another way: create a dummy Transform3d that represents a "camera" at the robot's center,
      // facing forward, and then apply that transform to QuestNav's robot pose to get a "camera"
      // pose.
      // This keeps the Vision subsystem's logic more consistent.

      // Let's use the second approach to keep Vision.java generic.
      // Assuming a dummy camera at (0,0,0) with no rotation relative to the robot.
      // This means QuestNav's robot pose IS the "camera" pose from QuestNav's perspective.
      // Vision.java will then subtract robotToCamera transform.

      // So, the pose returned by getRobotPose() is effectively the origin of the "QuestNav camera".
      // We need to apply the robotToCamera transform in reverse to get the robot pose.
      // The Vision subsystem's consumer expects a robot pose, so QuestNav.getRobotPose() is
      // directly usable.

      // Let's simplify and directly provide QuestNav's robot pose as a Pose3d observation.
      // This implies that Vision.java should treat QuestNav observations as *robot poses*, not
      // *camera poses*.
      // We'll need a way to distinguish this.

      // Okay, looking at the Vision.java again, it uses `observation.type() ==
      // PoseObservationType.QUESTNAV`.
      // This means QuestNav observations are treated differently. So we can directly pass robot
      // pose.
      Pose3d robotPose3d =
          new Pose3d(
              questNav.getRobotPose().getX(),
              questNav.getRobotPose().getY(),
              0,
              new Rotation3d(
                  0,
                  0,
                  questNav.getRobotPose().getRotation().getRadians())); // Assuming Z=0 for QuestNav

      observations.add(
          new PoseObservation(
              timestamp,
              robotPose3d,
              0.0, // Ambiguity is not provided by QuestNav directly, set to 0
              1, // Assume 1 tag, as it's a single pose source
              0.0, // Average tag distance not applicable
              VisionIO.PoseObservationType.QUESTNAV));
      inputs.poseObservations = observations.toArray(new PoseObservation[0]);
      inputs.latestTargetObservation =
          new TargetObservation(new Rotation2d(), new Rotation2d()); // Dummy
      inputs.tagIds = new int[0]; // No AprilTag IDs from QuestNav
    } else {
      inputs.poseObservations = new PoseObservation[0];
    }
    questNav.cleanUpQuestNavMessages();
  }
}
