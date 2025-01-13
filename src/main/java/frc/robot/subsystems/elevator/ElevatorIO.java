package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public void stop();

  public void resetRotationCount();

  public void setNeutralMode(NeutralModeValue mode);  
} 
