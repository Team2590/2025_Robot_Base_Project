package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;

public class ClimbIOTalonFX implements ClimbIO {
  public static final double MAX_ROTATIONS = -150;
  private final TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  public ClimbIOTalonFX(
      int canID, String canBus, int currentLimitAmps, boolean invert, boolean brake) {
    motor = new TalonFX(canID, canBus);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leaderRotationCount = motor.getPosition().getValueAsDouble();
    Logger.recordOutput("Climb Position", getRotationCount());
  }

  @Override
  public void run(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getRotationCount() {
    return motor.getPosition().getValueAsDouble();
  }

  public void resetRotationCount() {
    motor.setPosition(0);
  }

  public void setNeutralMode(NeutralModeValue mode) {
    motor.setNeutralMode(mode);
  }
}
