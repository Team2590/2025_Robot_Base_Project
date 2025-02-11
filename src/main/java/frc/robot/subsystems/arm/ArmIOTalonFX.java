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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;
import frc.robot.util.StickyFaultUtil;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX arm;
  private CANcoder armCancoder;
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 16);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.1);
  LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", -0.011);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", 1500); // 1500
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", 500); // 500
  LoggedTunableNumber MotionMagicJerk1 = new LoggedTunableNumber("Arm/MotionMagicJerk", 2000);
  LoggedTunableNumber ff = new LoggedTunableNumber("Arm/Feedforward", 0);
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
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fdb.FeedbackRemoteSensorID = cancoderID;
    MagnetSensorConfigs mag = new MagnetSensorConfigs();
    mag.MagnetOffset = magOffset;
    CANcoderConfiguration can = new CANcoderConfiguration();
    can.withMagnetSensor(mag);
    armCancoder.getConfigurator().apply(can);

    arm.getConfigurator().apply(cfg);
    mmv = new MotionMagicDutyCycle(0);

    position = arm.getPosition();
    velocity = arm.getVelocity();
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

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armabspos, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
    inputs.armabspos = armabspos.getValueAsDouble();
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
    // need to check the target position
    boolean withinAcceptedRange = NemesisMathUtil.isBetweenInclusive(
      position, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS
    );

    /**
     * if (its not between accepted setpoints){
     * if (elevator isn't in the right position){
     * sout(can't move)}
     * } else{
     * then you can move }
     * 
     * if (not within accepted setpoints and elevator isn't right )
     * 
     * if (within accepted or elevator is right )
     */

     if (withinAcceptedRange || checkElevator()){
      arm.setControl(mmv.withPosition(position));
     }
     else{
      System.out.println("CAN'T MOVE ARM, elevator not in valid position. ");
     }
  }

  public double getAbsolutePosition() {
    return armCancoder.getAbsolutePosition().getValueAsDouble();
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
    boolean withinAcceptedRange = NemesisMathUtil.isBetweenInclusive(
      getAbsolutePosition(), 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS
    );

    if (withinAcceptedRange || checkElevator()){
      arm.setControl(new VoltageOut(volts));
     }
     else{
      System.out.println("CAN'T MOVE ARM, elevator not in valid position. ");
     }
  }

  private boolean checkElevator(){
    return Robot.getRobotContainerInstance().getElevator().getRotationCount() > Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS;
  }

}
