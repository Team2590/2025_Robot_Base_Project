package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  static class IntakeIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public void stop();

  public void setVoltage(double voltage);

  public void updateInputs(IntakeIOInputs io);
}
