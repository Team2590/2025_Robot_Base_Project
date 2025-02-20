package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  class ClimbIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double rotationCount = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void setVoltage(double voltage);

  public void setVelocity(double speed);

  public void setPosition(double position);

  public void stop();
}
