package frc.robot.subsystems.peashooter;

import org.littletonrobotics.junction.AutoLog;

public interface PeaShooterIO {
  @AutoLog
  public static class PeaShooterIOInputs {
    public boolean leftFlywheelConnected = false;
    public double leftFlywheelVelocityRadPerSec = 0.0;
    public double leftFlywheelAppliedVolts = 0.0;
    public double leftFlywheelCurrentAmps = 0.0;
    public double leftFlywheelTempCelsius = 0.0;

    public boolean rightFlywheelConnected = false;
    public double rightFlywheelVelocityRadPerSec = 0.0;
    public double rightFlywheelAppliedVolts = 0.0;
    public double rightFlywheelCurrentAmps = 0.0;
    public double rightFlywheelTempCelsius = 0.0;

    public boolean hoodConnected = false;
    public double hoodPositionRad = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PeaShooterIOInputs inputs) {}

  /** Run the left flywheel at the specified voltage. */
  public default void setLeftVoltage(double volts) {}

  /** Run the right flywheel at the specified voltage. */
  public default void setRightVoltage(double volts) {}

  /** Run the hood at the specified voltage. */
  public default void setHoodVoltage(double volts) {}

  /** Run the left flywheel at the specified velocity. */
  public default void setLeftVelocity(double velocityRadPerSec) {}

  /** Run the right flywheel at the specified velocity. */
  public default void setRightVelocity(double velocityRadPerSec) {}

  /** Run the hood to the specified position. */
  public default void setHoodPosition(double positionRad) {}

  /** Stop all motors. */
  public default void stop() {}
}
