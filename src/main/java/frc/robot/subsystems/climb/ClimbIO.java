package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double positionRotations = 0.0;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void run(double voltage);

  public void setVoltage(double voltage);

  public void raise();

  public void lower();

  public void stop();

  public void resetRotationCount();

  public double getRotationCount();

  public void setNeutralMode(NeutralModeValue mode);
}
