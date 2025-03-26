package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.util.StickyFaultUtil;
import org.littletonrobotics.junction.Logger;

/**
 * @author Dhruv Shah, copied a bit from Vidur's 2024 code ngl
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX leader;
  private TalonFX follower;
  private LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", Constants.ElevatorConstantsLeonidas.kS);
  private LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", Constants.ElevatorConstantsLeonidas.kV);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.18);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 10);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
  private LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Elevator/cruiseVelocity", 3000);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/acceleration", 150);
  private LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/jerk", 750);
  private LoggedTunableNumber setPos = new LoggedTunableNumber("Elevator/setpointPos", 2000);
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

  public ElevatorIOTalonFX(
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
    talonFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .05;
    talonFXConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = .05;
    talonFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .05;
    StickyFaultUtil.clearMotorStickyFaults(leader, "Elevator Motor");

    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kG = kG.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

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

  public ElevatorIOTalonFX(
      int canID,
      String canBus,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      int followerCanID,
      String followerCanBus,
      boolean followerOpposeLeader) {
    this(canID, canBus, currentLimitAmps, invert, brake, reduction);
    follower = new TalonFX(followerCanID, followerCanBus);
    follower.setControl(new Follower(canID, followerOpposeLeader));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
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
    Logger.recordOutput("targetPositionForReal", position);
    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, position)) {
      var request = new MotionMagicVoltage(0);
      if (leader.getPosition().getValueAsDouble() < 0 || position < 0) {
        leader.setControl(request);
      } else {
        leader.setControl(request.withPosition(position));
      }
    } else {
      System.out.println("CAN'T MOVE ELEVATOR, safety check failed.");
    }
  }

  public void setPositionLoggedNumber() {
    if (SafetyChecker.isSafe(SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, setPos.get())) {
      var request = new MotionMagicVoltage(0);
      if (leader.getPosition().getValueAsDouble()
              < Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
          || setPos.get() < 0) {
        leader.setControl(request);
      } else {
        leader.setControl(request.withPosition(setPos.get()));
      }
    } else {
      System.out.println("CAN'T MOVE ELEVATOR, safety check failed.");
    }
  }

  public double getTunableNumber() {
    return setPos.get();
  }

  @Override
  public double getTargetPosition() {
    return leader.getPosition().getValueAsDouble();
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
  public void setVoltage(VoltageOut volts) {

    // NOTE- without a setpoint, we can't check if the elevator movement is safe. The following will
    // only check the current positions safety, which should never be the case.
    if (SafetyChecker.isSafe(
        SafetyChecker.MechanismType.ELEVATOR_MOVEMENT, position.getValueAsDouble())) {
      leader.setControl(volts);
    } else {
      System.out.println("CAN'T MOVE ELEVATOR, arm not in valid position");
    }
  }

  public double getRotationCounts() {
    return leader.getPosition().getValueAsDouble();
  }
}
