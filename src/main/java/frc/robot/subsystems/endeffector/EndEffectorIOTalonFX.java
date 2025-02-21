package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.EndEffectorConstantsLeonidas;
import frc.robot.util.LoggedTunableNumber;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX leader;
  private LoggedTunableNumber voltageTunableNumber =
      new LoggedTunableNumber("EndEffector/voltage", 6);
  LoggedTunableNumber ff = new LoggedTunableNumber("Arm/Feedforward", 0);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  private double reduction;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Temperature> tempCelsius;
  private double statorCurrentAmps;
  private AnalogInput proxInput = new AnalogInput(EndEffectorConstantsLeonidas.proxSensor_ID);

  public EndEffectorIOTalonFX(
      int canID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    leader = new TalonFX(canID, canBus);

    cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    leader.getConfigurator().apply(cfg);

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVoltage = leader.getMotorVoltage();
    supplyCurrent = leader.getSupplyCurrent();
    torqueCurrent = leader.getTorqueCurrent();
    tempCelsius = leader.getDeviceTemp();
    statorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    this.reduction = reduction;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius
        // cancoderPosition,
        // cancoderAbsPosition
        );
    leader.optimizeBusUtilization(0, 1);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
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
    inputs.rotationCount = leader.getPosition().getValueAsDouble();
    statorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.proxValue = proxInput.getValue();
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void stop() {
    leader.set(0);
  }

  @Override
  public void setVelocity(double velocity) {
    leader.set(velocity);
  }

  public double getProxValue(){
    return proxInput.getValue();
  }
}
