package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private AbsoluteEncoderEmulator encoderEmulator;
  private ProfiledPIDController controller;
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 3);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/maxVelocity", 1000); // 1500
  LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/maxAcceleration", 50); // 500
  LoggedTunableNumber setPos = new LoggedTunableNumber("Arm/setpointPos", 0);
  private double reduction;
  private StatusSignal<Angle> armabspos; // TODO
  private StatusSignal<Angle> position; // TODO
  private StatusSignal<AngularVelocity> velocity; // TODO
  private StatusSignal<Voltage> appliedVoltage; // TODO
  private StatusSignal<Current> supplyCurrent; // TODO
  private StatusSignal<Current> torqueCurrent; // TODO
  private StatusSignal<Temperature> tempCelsius; // TODO
  private boolean runningSetPosition;
  private double setpoint;
  private ArmFeedforward feedforward;

  public ArmIOTalonFX(
    int motorCanID,
    String canBus,
    int currentLimitAmps,
    boolean invert,
    boolean brake,
    double reduction,
    int cancoderID,
    double magOffset,
    double sensor_reduction
  ) {
    this.reduction = reduction;
    arm = new TalonFX(motorCanID, canBus);
    StickyFaultUtil.clearMotorStickyFaults(arm, "Arm Motor");
    encoderEmulator = new AbsoluteEncoderEmulator(arm, 0, sensor_reduction);
    
    controller = new ProfiledPIDController(kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    position = arm.getPosition(); // TODO
    appliedVoltage = arm.getMotorVoltage(); // TODO
    supplyCurrent = arm.getSupplyCurrent(); // TODO
    torqueCurrent = arm.getTorqueCurrent(); // TODO
    tempCelsius = arm.getDeviceTemp(); // TODO

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

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armabspos, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
    inputs.armabspos = armabspos.getValueAsDouble(); // TODO
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction; // TODO
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction; // TODO
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble(); // TODO
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble(); // TODO
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble(); // TODO
    inputs.tempCelsius = tempCelsius.getValueAsDouble(); // TODO
    inputs.rotationCount = arm.getPosition().getValueAsDouble(); // TODO
    updateTunableNumbers();
    if (runningSetPosition) arm.setVoltage(controller.calculate(encoderEmulator.getAbsolutePosition(), setpoint));
  }

  public void setPosition(double position) {
    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, position)) {
      setpoint = position;
      runningSetPosition = true;
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }

  public void setPositionLoggedNumber() {
    setPosition(setPos.get());
  }

  public double getAbsolutePosition() {
    return encoderEmulator.getAbsolutePosition();
  }

  public void stop() {
    runningSetPosition = false;
    arm.stopMotor();
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0)) controller.setP(kP.get());
    if (kI.hasChanged(0)) controller.setI(kI.get());
    if (kD.hasChanged(0)) controller.setD(kD.get());
    if (maxVelocity.hasChanged(0) || maxAcceleration.hasChanged(0)) controller.setConstraints(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  }

  public void setPower(DutyCycleOut power) {
    runningSetPosition = false;
    arm.setControl(power);
  }

  @Override
  public void setVoltage(double volts) {
    runningSetPosition = false;
    double armPos = getAbsolutePosition();

    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, armPos)) {
      arm.setControl(new VoltageOut(volts));
    } else {
      System.out.println("CAN'T MOVE ARM, safety check failed.");
    }
  }
}
