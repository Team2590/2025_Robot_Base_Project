package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX leader = new TalonFX(0);
  private TalonFX follower = new TalonFX(0);

  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  public void setSpeed(double speed) {
    leader.set(speed);
    follower.set(speed);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.followerAppliedVolts = follower.getMotorVoltage().getValueAsDouble();
  }
}
