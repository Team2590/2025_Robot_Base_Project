package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double rotationCount = 0.0;
  }

  public void updateInputs(ElevatorIOInputs io);

  public void updateTunableNumbers();

  public void setPosition(double position);

  public double getTargetPosition();

  public void stop();

  public void resetRotationCount();

  public void setNeutralMode(NeutralModeValue mode);

  /** Run open loop at the specified voltage. */
  public default void setVoltage(VoltageOut volts) {}
}
