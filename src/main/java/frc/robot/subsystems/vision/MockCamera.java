package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.Random;

public class MockCamera implements Runnable {
  private final Random random = new Random();
  private final Drive drive;
  private final String camName;
  private final Pose2d base;
  private final Timer timer;

  public MockCamera(Drive drive, String camName, Pose2d base) {
    this.drive = drive;
    this.camName = camName;
    this.base = base;
    this.timer = new Timer();
    this.timer.start();
  }

  @Override
  public void run() {
    while (true) {

      try {
        double x = base.getX() + random.nextGaussian() * 0.1;
        double y = base.getY() + random.nextGaussian() * 0.1;
        double rot = base.getRotation().getDegrees() + random.nextGaussian() * 2;
        Pose2d pose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(rot));
        updatePose(pose);
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        System.out.println("Thread : " + camName + " interrupted");
        return;
      }
    }
  }

  public void updatePose(Pose2d pose) {
    double timestamp = Timer.getFPGATimestamp();
    drive.addVisionMeasurement(pose, timestamp);
    System.out.println("[" + camName + "] New pose: " + pose + " at timestamp: " + timestamp);
  }
}
