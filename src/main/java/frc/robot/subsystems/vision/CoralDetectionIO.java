package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectionIO {
  @AutoLog
  public static class CoralDetectionIOInputs {
    public double coralYaw;
    public Rotation2d coralRotation;
    public Transform2d robotToCoral;
    public Pose2d coralPose;
  }

  public void updateInputs(CoralDetectionIOInputs inputs);
}
