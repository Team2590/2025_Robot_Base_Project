package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX talon;
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final double reduction;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakeIOTalonFX(
      int canID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    this.reduction = reduction;
    talon = new TalonFX(canID, canBus);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

    talon.optimizeBusUtilization(0, 1.0);
  }

  public void stop() {
    talon.setControl(neutralOut);
  }

  public void setVoltage(double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
}
