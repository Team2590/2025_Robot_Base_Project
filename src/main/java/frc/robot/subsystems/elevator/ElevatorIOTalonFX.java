package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTunableNumber;


/**
 * @author Dhruv Shah, copied a bit from Vidur ngl
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX leader = new TalonFX(0);
  private CANcoder cancoder = new CANcoder(0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0.25);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.12);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", 0.01);
  private LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 4.8);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0.1);
  private LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber(null);
  private LoggedTunableNumber acceleration = new LoggedTunableNumber(null);
  private LoggedTunableNumber jerk = new LoggedTunableNumber(null);
  private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
  private Slot0Configs slot0Configs = talonFXConfig.Slot0;
  private MotionMagicConfigs motionMagicConfigs = talonFXConfig.MotionMagic;

  public ElevatorIOTalonFX() {
    slot0Configs.kS = kS.get();  
    slot0Configs.kV = kV.get();  
    slot0Configs.kG = kG.get();  
    slot0Configs.kP = kP.get();  
    slot0Configs.kI = kI.get(); 
    slot0Configs.kD = kD.get();

    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity.get();
    motionMagicConfigs.MotionMagicAcceleration = acceleration.get();
    motionMagicConfigs.MotionMagicJerk = jerk.get();

    talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonFXConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();

    leader.getConfigurator().apply(talonFXConfig);
    leader.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateInputs(ElevatorIOInputs io) {
    io.rotationCount = leader.getPosition().getValueAsDouble();
    io.absRotationCount = cancoder.getAbsolutePosition().getValueAsDouble();
  }

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

  public void setPosition(double position) {
    var request = new MotionMagicVoltage(0);
    leader.setControl(request.withPosition(position));
  }

  public void stop() {
    leader.stopMotor();
  }
}
