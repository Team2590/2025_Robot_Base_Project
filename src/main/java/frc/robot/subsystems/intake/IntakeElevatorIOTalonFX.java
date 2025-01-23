package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.util.LoggedTunableNumber;

public class IntakeElevatorIOTalonFX implements IntakeElevatorIO {
  private TalonFX leader;
  private LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 16);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.1);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", -0.011);
  private LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("IntakeArm/cruiseVelocity", 0);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("IntakeArm/acceleration", 0);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("IntakeArm/jerk", 0);
  private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
  private Slot0Configs slot0Configs = talonFXConfig.Slot0;
  private MotionMagicConfigs motionMagicConfigs = talonFXConfig.MotionMagic;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Temperature> tempCelsius;
  private double reduction;
  private CANcoder cancoder;
  private final StatusSignal<Angle> cancoderPosition;
  private final StatusSignal<Angle> cancoderAbsPosition;

  public IntakeElevatorIOTalonFX(
      int canID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    leader = new TalonFX(canID, canBus);

    var mag = new MagnetSensorConfigs();

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.withMagnetSensor(mag);
    cancoder.getConfigurator().apply(cancoderConfig);

    cancoderPosition = cancoder.getPosition();
    cancoderAbsPosition = cancoder.getAbsolutePosition();

    talonFXConfig.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFXConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kG = kG.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
    motionMagicConfigs.MotionMagicAcceleration = acceleration.get();
    motionMagicConfigs.MotionMagicJerk = jerk.get();

    leader.getConfigurator().apply(talonFXConfig);

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVoltage = leader.getMotorVoltage();
    supplyCurrent = leader.getSupplyCurrent();
    torqueCurrent = leader.getTorqueCurrent();
    tempCelsius = leader.getDeviceTemp();
    this.reduction = reduction;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        cancoderPosition,
        cancoderAbsPosition);
    leader.optimizeBusUtilization(0, 1);
    cancoder.optimizeBusUtilization(0, 1);
  }

  @Override
  public void updateInputs(IntakeElevatorIOInputs inputs) {
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
      motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
      applyConfig = true;
    }

    if (acceleration.hasChanged(0)) {
      motionMagicConfigs.MotionMagicAcceleration = acceleration.get();
      applyConfig = true;
    }

    if (jerk.hasChanged(0)) {
      motionMagicConfigs.MotionMagicJerk = jerk.get();
      applyConfig = true;
    }

    if (applyConfig) {
      leader.getConfigurator().apply(talonFXConfig);
    }
  }

  @Override
  public void setPosition(double position) {
    var request = new MotionMagicVoltage(0);
    if (leader.getPosition().getValueAsDouble() < 0 || position < 0) {
      leader.setControl(request);
    } else {
      leader.setControl(request.withPosition(position));
    }
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void resetRotationCount() {
    leader.setPosition(0);
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    leader.setNeutralMode(mode);
  }
}
