package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public double rotationCount = 0.0;
    public double absRotationCount = 0.0;
  }

  public void updateInputs(ElevatorIOInputs io);

  public void updateTunableNumbers();

  public void setPosition(double position);

  public void stop();
}
