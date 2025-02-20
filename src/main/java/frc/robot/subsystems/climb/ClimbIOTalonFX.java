package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX talon;
  private final TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  private double reduction;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Temperature> tempCelsius;
  private Slot0Configs slot0Configs;
  private LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", Constants.ElevatorConstantsLeonidas.kS);
  private LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", Constants.ElevatorConstantsLeonidas.kV);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.18);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 16);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
  private LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Elevator/cruiseVelocity", 3000);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/acceleration", 300);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/jerk", 750);

  public ClimbIOTalonFX(
      int canId,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    talon = new TalonFX(canId, canBus);
    cfg = new TalonFXConfiguration();
    slot0Configs = cfg.Slot0;
    mm = cfg.MotionMagic;

    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kG = kG.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    mm.MotionMagicCruiseVelocity = cruiseVelocity.get();
    mm.MotionMagicAcceleration = acceleration.get();
    mm.MotionMagicJerk = jerk.get();

    cfg.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    talon.getConfigurator().apply(cfg);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    this.reduction = reduction;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius
        // cancoderPosition,
        // cancoderAbsPosition
        );
    talon.optimizeBusUtilization(0, 1);
  }

  public void updateInputs(ClimbIOInputs inputs) {
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
    inputs.rotationCount = talon.getPosition().getValueAsDouble();
  }

  @Override
  public void updateTunableNumbers() {
    boolean applyConfig = false;

    if (kP.hasChanged(0)) {
      slot0Configs.kP = kP.get();
      applyConfig = true;
    }

    if (kI.hasChanged(0)) {
      slot0Configs.kI = kI.get();
      applyConfig = true;
    }

    if (kD.hasChanged(0)) {
      slot0Configs.kD = kD.get();
      applyConfig = true;
    }

    if (kS.hasChanged(0)) {
      slot0Configs.kS = kS.get();
      applyConfig = true;
    }

    if (kV.hasChanged(0)) {
      slot0Configs.kV = kV.get();
      applyConfig = true;
    }

    if (kG.hasChanged(0)) {
      slot0Configs.kG = kG.get();
      applyConfig = true;
    }

    if (cruiseVelocity.hasChanged(0)) {
      mm.MotionMagicCruiseVelocity = cruiseVelocity.get();
      applyConfig = true;
    }

    if (acceleration.hasChanged(0)) {
      mm.MotionMagicAcceleration = acceleration.get();
      applyConfig = true;
    }

    if (jerk.hasChanged(0)) {
      mm.MotionMagicJerk = jerk.get();
      applyConfig = true;
    }

    if (applyConfig) {
      talon.getConfigurator().apply(cfg);
    }
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setVoltage(voltage);
  }

  @Override
  public void stop() {
    talon.set(0);
  }

  @Override
  public void setVelocity(double velocity) {
    talon.set(velocity);
  }

  @Override
  public void setPosition(double position) {
    talon.setControl(mmv.withPosition(position));
  }
}
