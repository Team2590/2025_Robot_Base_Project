package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SafetyChecker;
import frc.robot.util.StickyFaultUtil;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX arm;
  private CANcoder armCancoder;
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 3);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", Constants.ArmConstantsLeonidas.DEFAULT_CRUISE_VELOCITY);
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", Constants.ArmConstantsLeonidas.DEFAULT_ACCELERATION);
  LoggedTunableNumber MotionMagicJerk1 = new LoggedTunableNumber("Arm/MotionMagicJerk", Constants.ArmConstantsLeonidas.DEFAULT_JERK);
  LoggedTunableNumber setPos = new LoggedTunableNumber("Arm/setpointPos", 0);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  private double reduction;
  private StatusSignal<Angle> armpos;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Temperature> tempCelsius;

  public ArmIOTalonFX(
      int motorCanID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      int cancoderID,
      double magOffset,
      double sensor_reduction) {

    this.reduction = reduction;
    arm = new TalonFX(motorCanID, canBus);
    armCancoder = new CANcoder(cancoderID, canBus);
    StickyFaultUtil.clearCancoderStickyFaults(armCancoder, "Arm Cancoder");
    StickyFaultUtil.clearMotorStickyFaults(arm, "Arm Motor");
    /* configurations for the arm motor */
    cfg = new TalonFXConfiguration();
    // cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;
    cfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = .05;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .05;

    /* Configure current limits */
    mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity1.get();
    mm.MotionMagicAcceleration = MotionMagicAcceleration1.get();
    mm.MotionMagicJerk = MotionMagicJerk1.get();

    slot0 = cfg.Slot0;
    slot0.kP = kP.get();
    slot0.kD = kD.get();
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    FeedbackConfigs fdb = cfg.Feedback;
    // fdb.RotorToSensorRatio = sensor_reduction;
    fdb.SensorToMechanismRatio = 2.666 / 2.5;
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fdb.FeedbackRemoteSensorID = cancoderID;
    MagnetSensorConfigs mag = new MagnetSensorConfigs();
    mag.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    mag.MagnetOffset = magOffset;
    mag.AbsoluteSensorDiscontinuityPoint = 0.5;
    CANcoderConfiguration can = new CANcoderConfiguration();
    can.withMagnetSensor(mag);
    armCancoder.getConfigurator().apply(can);

    arm.getConfigurator().apply(cfg);
    mmv = new MotionMagicDutyCycle(0);

    position = arm.getPosition();
    velocity = armCancoder.getVelocity();
    appliedVoltage = arm.getMotorVoltage();
    supplyCurrent = arm.getSupplyCurrent();
    torqueCurrent = arm.getTorqueCurrent();
    tempCelsius = arm.getDeviceTemp();
    armpos = arm.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armpos,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armpos, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
    inputs.armpos = armpos.getValueAsDouble();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.rotationCount = arm.getPosition().getValueAsDouble();
    updateTunableNumbers();
  }

  public void setPosition(double position) {

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, position)) {
      arm.setControl(mmv.withPosition(position));
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }

  public void setPositionLoggedNumber() {

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, setPos.get())) {
      arm.setControl(mmv.withPosition(setPos.get()));
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }

  public double getAbsolutePosition() {
    return armCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTunableNumber() {
    return setPos.get();
  }

  public void stop() {
    arm.stopMotor();
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0)) {
      slot0.kP = kP.get();
      arm.getConfigurator().apply(cfg);
    }

    if (kD.hasChanged(0)) {
      slot0.kD = kD.get();
      arm.getConfigurator().apply(cfg);
    }
    if (MotionMagicCruiseVelocity1.hasChanged(0)) {
      mm.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity1.get();
      arm.getConfigurator().apply(cfg);
    }

    if (MotionMagicAcceleration1.hasChanged(0)) {
      mm.MotionMagicAcceleration = MotionMagicAcceleration1.get();
      arm.getConfigurator().apply(cfg);
    }
    if (MotionMagicJerk1.hasChanged(0)) {
      mm.MotionMagicJerk = MotionMagicJerk1.get();
      arm.getConfigurator().apply(cfg);
    }
  }

  public void setPower(DutyCycleOut power) {
    arm.setControl(power);
  }

  @Override
  public void setVoltage(double volts) {
    double armPos = getAbsolutePosition();

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, armPos)) {
      arm.setControl(new VoltageOut(volts));
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }

  public void setMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
    mm.MotionMagicCruiseVelocity = cruiseVelocity;
    mm.MotionMagicAcceleration = acceleration;
    mm.MotionMagicJerk = jerk;
    arm.getConfigurator().apply(cfg);
  }
}
