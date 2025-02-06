package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
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

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default void stopMotor() {}
}
