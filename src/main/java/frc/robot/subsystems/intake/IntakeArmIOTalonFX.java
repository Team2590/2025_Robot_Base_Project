package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.util.SafetyChecker;
import frc.robot.util.SafetyChecker.MechanismType;

public class IntakeArmIOTalonFX implements IntakeArmIO {
  private TalonFX leader;
  private LoggedTunableNumber kP = new LoggedTunableNumber("IntakeArm/kP", 1.4);
  private LoggedTunableNumber kD = new LoggedTunableNumber("IntakeArm/kD", 0);
  private LoggedTunableNumber kG = new LoggedTunableNumber("IntakeArm/kG", 0.075);
  private LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("IntakeArm/cruiseVelocity", 1500);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("IntakeArm/acceleration", 25);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("IntakeArm/jerk", 1500);
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

  public IntakeArmIOTalonFX(
      int canID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    leader = new TalonFX(canID, canBus);

    talonFXConfig.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFXConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.Feedback.SensorToMechanismRatio = reduction;

    slot0Configs.kP = kP.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kG = kG.get();
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
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
    leader.optimizeBusUtilization(0, 1);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
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
    if (kP.hasChanged(0)) {
      slot0Configs.kP = kP.get();
      leader.getConfigurator().apply(talonFXConfig);
    }

    if (kD.hasChanged(0)) {
      slot0Configs.kD = kD.get();
      leader.getConfigurator().apply(talonFXConfig);
    }

    if (cruiseVelocity.hasChanged(0)) {
      motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
      leader.getConfigurator().apply(talonFXConfig);
    }

    if (acceleration.hasChanged(0)) {
      motionMagicConfigs.MotionMagicAcceleration = acceleration.get();
      leader.getConfigurator().apply(talonFXConfig);
    }

    if (jerk.hasChanged(0)) {
      motionMagicConfigs.MotionMagicJerk = jerk.get();
      leader.getConfigurator().apply(talonFXConfig);
    }

    if (kG.hasChanged(0)) {
      slot0Configs.kG = kG.get();

      leader.getConfigurator().apply(talonFXConfig);
    }
  }

  @Override
  public void setPosition(double position) {
    var request = new MotionMagicDutyCycle(0);
    // less than 0.05 because when set to 0, it is never exactly zero
    if (SafetyChecker.isSafe(MechanismType.INTAKE_MOVEMENT, position)) {
      if (leader.getPosition().getValueAsDouble() < -0.05 || position < 0) {
        leader.setControl(request);
      } else {
        leader.setControl(request.withPosition(position));
      }
    } else {
      System.out.println("CAN'T MOVE INTAKE ARM, SAFETY CHECK FAILED");
    }
  }

  public void setInitPosition() {
    leader.setPosition(Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS);
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

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public double getRotationCount() {
    return leader.getPosition().getValueAsDouble();
  }
}
