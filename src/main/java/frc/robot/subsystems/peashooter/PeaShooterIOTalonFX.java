package frc.robot.subsystems.peashooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.Constants.PeaShooterConstants;

public class PeaShooterIOTalonFX implements PeaShooterIO {
  private final TalonFX leftFlywheel;
  private final TalonFX rightFlywheel;
  private final TalonFX hood;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  // Inputs
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrent;
  private final StatusSignal<Temperature> leftTemp;

  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;
  private final StatusSignal<Temperature> rightTemp;

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;
  private final StatusSignal<Temperature> hoodTemp;

  public PeaShooterIOTalonFX() {
    leftFlywheel = new TalonFX(PeaShooterConstants.LEFT_FLYWHEEL_ID, PeaShooterConstants.CANBUS);
    rightFlywheel = new TalonFX(PeaShooterConstants.RIGHT_FLYWHEEL_ID, PeaShooterConstants.CANBUS);
    hood = new TalonFX(PeaShooterConstants.HOOD_ID, PeaShooterConstants.CANBUS);

    // Configure Left Flywheel
    var flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kP = PeaShooterConstants.FLYWHEEL_kP;
    flywheelConfig.Slot0.kI = PeaShooterConstants.FLYWHEEL_kI;
    flywheelConfig.Slot0.kD = PeaShooterConstants.FLYWHEEL_kD;
    flywheelConfig.Slot0.kS = PeaShooterConstants.FLYWHEEL_kS;
    flywheelConfig.Slot0.kV = PeaShooterConstants.FLYWHEEL_kV;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = PeaShooterConstants.CURRENT_LIMIT_AMPS;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted =
        PeaShooterConstants.INVERT_LEFT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftFlywheel.getConfigurator().apply(flywheelConfig);

    // Configure Right Flywheel
    flywheelConfig.MotorOutput.Inverted =
        PeaShooterConstants.INVERT_RIGHT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightFlywheel.getConfigurator().apply(flywheelConfig);

    // Configure Hood
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = PeaShooterConstants.HOOD_kP;
    hoodConfig.Slot0.kI = PeaShooterConstants.HOOD_kI;
    hoodConfig.Slot0.kD = PeaShooterConstants.HOOD_kD;
    hoodConfig.CurrentLimits.StatorCurrentLimit = PeaShooterConstants.CURRENT_LIMIT_AMPS;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted =
        PeaShooterConstants.INVERT_HOOD
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    hoodConfig.Feedback.SensorToMechanismRatio = PeaShooterConstants.HOOD_REDUCTION;
    hood.getConfigurator().apply(hoodConfig);

    // Signals
    leftVelocity = leftFlywheel.getVelocity();
    leftAppliedVolts = leftFlywheel.getMotorVoltage();
    leftCurrent = leftFlywheel.getStatorCurrent();
    leftTemp = leftFlywheel.getDeviceTemp();

    rightVelocity = rightFlywheel.getVelocity();
    rightAppliedVolts = rightFlywheel.getMotorVoltage();
    rightCurrent = rightFlywheel.getStatorCurrent();
    rightTemp = rightFlywheel.getDeviceTemp();

    hoodPosition = hood.getPosition();
    hoodAppliedVolts = hood.getMotorVoltage();
    hoodCurrent = hood.getStatorCurrent();
    hoodTemp = hood.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        leftTemp,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent,
        rightTemp,
        hoodPosition,
        hoodAppliedVolts,
        hoodCurrent,
        hoodTemp);
  }

  @Override
  public void updateInputs(PeaShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        leftTemp,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent,
        rightTemp,
        hoodPosition,
        hoodAppliedVolts,
        hoodCurrent,
        hoodTemp);

    inputs.leftFlywheelConnected = BaseStatusSignal.isAllGood(leftVelocity, leftAppliedVolts);
    inputs.leftFlywheelVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftFlywheelAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftFlywheelCurrentAmps = leftCurrent.getValueAsDouble();
    inputs.leftFlywheelTempCelsius = leftTemp.getValueAsDouble();

    inputs.rightFlywheelConnected = BaseStatusSignal.isAllGood(rightVelocity, rightAppliedVolts);
    inputs.rightFlywheelVelocityRadPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightFlywheelAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightFlywheelCurrentAmps = rightCurrent.getValueAsDouble();
    inputs.rightFlywheelTempCelsius = rightTemp.getValueAsDouble();

    inputs.hoodConnected = BaseStatusSignal.isAllGood(hoodPosition, hoodAppliedVolts);
    inputs.hoodPositionRad = Units.rotationsToRadians(hoodPosition.getValueAsDouble());
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
    inputs.hoodTempCelsius = hoodTemp.getValueAsDouble();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftFlywheel.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setRightVoltage(double volts) {
    rightFlywheel.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setHoodVoltage(double volts) {
    hood.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLeftVelocity(double velocityRadPerSec) {
    leftFlywheel.setControl(
        velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setRightVelocity(double velocityRadPerSec) {
    rightFlywheel.setControl(
        velocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setHoodPosition(double positionRad) {
    hood.setControl(positionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void stop() {
    leftFlywheel.setControl(voltageRequest.withOutput(0));
    rightFlywheel.setControl(voltageRequest.withOutput(0));
    hood.setControl(voltageRequest.withOutput(0));
  }
}
