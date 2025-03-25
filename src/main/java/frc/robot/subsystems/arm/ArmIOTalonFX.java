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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SafetyChecker;
import frc.robot.util.StickyFaultUtil;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX arm;
  private CANcoder armCancoder;
  private ArmPositionManager armPositionManager;
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 8);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", .15);
  LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.15);
  LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", 0);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", 1500); // 1500
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", 50); // 500
  LoggedTunableNumber MotionMagicJerk1 = new LoggedTunableNumber("Arm/MotionMagicJerk", 100);
  LoggedTunableNumber ff = new LoggedTunableNumber("Arm/Feedforward", 0);
  LoggedTunableNumber setPos = new LoggedTunableNumber("Arm/setpointPos", 0);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  private double reduction;
  private StatusSignal<Angle> armabspos;
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
    armPositionManager = new ArmPositionManager(reduction, 2, magOffset);
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
    slot0.kI = kI.get();
    slot0.kD = kD.get();
    slot0.kS = kS.get();
    slot0.kG = kG.get();
    slot0.kV = kV.get();
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.RotorToSensorRatio = sensor_reduction;
    // fdb.SensorToMechanismRatio = 1.6;
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fdb.FeedbackRemoteSensorID = cancoderID;
    MagnetSensorConfigs mag = new MagnetSensorConfigs();
    mag.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    mag.MagnetOffset = magOffset;
    mag.AbsoluteSensorDiscontinuityPoint = 0.9;
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
    armabspos = armCancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armabspos,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armabspos, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

    // Update rotation tracking with current position
    double currentAbsPos = armabspos.getValueAsDouble();
    double currentAngle = armPositionManager.updateRotationTracking(currentAbsPos);

    inputs.armabspos = currentAbsPos;
    inputs.positionRads = Units.degreesToRadians(currentAngle);
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.rotationCount = armPositionManager.getRotationCount();

    updateTunableNumbers();
  }

  /**
   * Sets the arm position to a specific angle in degrees.
   *
   * <p>The angle system follows a clock face convention: - 0°: Arm pointing straight up (12 o'clock
   * position) - 90°: Arm pointing to the right (3 o'clock position) - 180°: Arm pointing straight
   * down (6 o'clock position) - 270°: Arm pointing to the left (9 o'clock position)
   *
   * <p>The method automatically: 1. Checks if the move is safe within rotation limits 2. Calculates
   * the shortest path to reach the target 3. Converts the angle to the appropriate CANcoder
   * position 4. Moves the arm to the target position
   *
   * @param targetDegrees The desired arm position in degrees (0-360)
   * @throws IllegalArgumentException if targetDegrees is outside 0-360 range
   */
  @Override
  public void setPosition(double targetDegrees) {
    // First try to find a safe path (either shortest or long way around if shortest isn't safe)
    double safePath = armPositionManager.calculateSafePath(targetDegrees);

    if (safePath >= 0) {
      // Convert the safe path angle to CANcoder position
      double targetPos = armPositionManager.degreesToCancoderPosition(safePath);

      // Get the current CANcoder position
      double currentPos = armabspos.getValueAsDouble();

      // Calculate the direction of movement
      double direction = targetPos - currentPos;

      // If the direction is negative, add 1 to ensure Motion Magic moves the long way around
      if (direction < 0) {
        targetPos += 1.0;
      }

      // Set the target position
      arm.setControl(mmv.withPosition(targetPos));
    } else {
      System.out.println("CAN'T MOVE ARM, no safe path exists to target position.");
    }
  }

  public void setPositionLoggedNumber() {

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, setPos.get())) {
      arm.setControl(mmv.withPosition(setPos.get()));
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }

  @Override
  public double getAbsolutePosition() {
    return armPositionManager.getNormalizedAngleDegrees();
  }

  public void stop() {
    arm.stopMotor();
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0)) {
      slot0.kP = kP.get();
      arm.getConfigurator().apply(cfg);
    }

    if (kI.hasChanged(0)) {
      slot0.kI = kI.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kD.hasChanged(0)) {
      slot0.kD = kD.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kS.hasChanged(0)) {
      slot0.kS = kS.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kV.hasChanged(0)) {
      slot0.kV = kV.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kG.hasChanged(0)) {
      slot0.kG = kG.get();
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
    if (ff.hasChanged(0)) {
      mmv.FeedForward = ff.get();
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
}
