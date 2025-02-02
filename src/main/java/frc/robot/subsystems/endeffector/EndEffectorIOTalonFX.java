package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTunableNumber;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX motor;
  private LoggedTunableNumber voltageTunableNumber =
      new LoggedTunableNumber("EndEffector/voltage", 0);
  private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

  public EndEffectorIOTalonFX() {
    motor = new TalonFX(14, "Takeover");
    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(talonFXConfig);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorSpeed = motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltageTunableNumber.get());
  }

  @Override
  public void stopMotor() {
    motor.set(0);
  }
}
