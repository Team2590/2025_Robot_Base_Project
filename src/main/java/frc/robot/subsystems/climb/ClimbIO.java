package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public boolean connected = false;
    public double leaderPositionRads = 0.0;
    public double followerPositionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double leaderRotationCount = 0.0;
    public double followerRotationCount = 0.0;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void run(double voltage);

  public void setVoltage(double voltage);

  public void stop();

  public void resetRotationCount();

  public double getRotationCount();

  public void setNeutralMode(NeutralModeValue mode);
}
