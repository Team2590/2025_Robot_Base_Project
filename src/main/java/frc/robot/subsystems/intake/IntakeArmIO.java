package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double rotationCount = 0.0;
  }

  public void updateInputs(IntakeArmIOInputs io);

  public void updateTunableNumbers();

  public void setPosition(double position);

  public void stop();

  public void resetRotationCount();

  public void setNeutralMode(NeutralModeValue mode);
}
