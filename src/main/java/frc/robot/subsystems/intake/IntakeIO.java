package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
  }

  public void stop();

  public void setSpeed(double speed);

  public void updateInputs(IntakeIOInputs io);
}
