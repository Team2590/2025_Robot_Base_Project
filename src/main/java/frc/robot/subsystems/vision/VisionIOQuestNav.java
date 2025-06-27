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

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.QuestNav;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOQuestNav implements VisionIO {
  public static record QuestConfig(String name) {}

  protected final List<QuestThread> questThreads = new ArrayList<>();
  private final Object observationLock = new Object();
  private List<PoseObservation> latestPoseObservations = new ArrayList<>();
  private TargetObservation latestTargetObservation =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  /**
   * Creates a new VisionIOPhotonVision with multiple cameras.
   *
   * @param cameraConfigs List of camera configurations (name and transform)
   */
  public VisionIOQuestNav(QuestConfig questConfig) {
    QuestThread thread = new QuestThread(questConfig.name());
    questThreads.add(thread);

    // Fixes race condition with Alerts being created in PhotonCamera
    // which causes ConcurrentModificationException.
    thread.start();
        try {
            Thread.sleep(20);
        } catch (InterruptedException ignore) {
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    synchronized (observationLock) {
      // Update connected status - true if any camera is connected
      inputs.connected = questThreads.stream().anyMatch(QuestThread::isConnected);

      // Copy latest observations to inputs
      inputs.poseObservations = latestPoseObservations.toArray(new PoseObservation[0]);
      inputs.latestTargetObservation = latestTargetObservation;

      // Clear for next update
      latestPoseObservations.clear();
    }
  }

  protected class QuestThread extends Thread {
    private volatile boolean connected = false;
    private QuestNav questNav;
    private Pose3d robotPose;
    private Pose3d initialRobotPose;

    public QuestThread(String name) {
      super("Vision-" + name);
      setDaemon(true);
      questNav = new QuestNav();
      initialRobotPose = /* find out how to get initial pose using photonvision */ new Pose3d();
      questNav.setPose(initialRobotPose.toPose2d());
      robotPose = new Pose3d(questNav.getPose()).transformBy(new Transform3d(0, 0, initialRobotPose.getZ(), new Rotation3d()));
    }

    public boolean isConnected() {
      return connected;
    }

    public void updateRobotPose(){
        robotPose = new Pose3d(questNav.getPose()).transformBy(new Transform3d(0, 0, initialRobotPose.getZ(), new Rotation3d()));
    }

    @Override
    public void run() {
        if(questNav.isConnected() && questNav.isTracking()){
            updateRobotPose();
            double timestamp = questNav.getDataTimestamp();
            latestPoseObservations.add(
                new PoseObservation(
                    timestamp, 
                    robotPose, 
                    0.01, 
                    0, 
                    0.1, 
                    PoseObservationType.QUESTNAV)
            );
            // TODO: FIX POSE OBSERVATION TO FIT WITH QUESTNAV INSTEAD OF PUTTING DUMMY VALUES
        }
    }
  }
}
