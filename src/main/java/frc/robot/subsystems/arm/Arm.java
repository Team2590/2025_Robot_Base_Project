package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO arm;
  private double setpointTolerance = 0.01;
  private double setpoint;

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    arm = io;
  }

  @Override
  public void periodic() {
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Log current position and target position
    // Logger.recordOutput("Arm/CurrentPosition", inputs.armpos);
    Logger.recordOutput("Arm/TargetPosition", setpoint);
  }

  /** Run open loop at the specified voltage. */
  public Command setPositionCommand(double setpoint) {
    return runOnce(() -> arm.setPosition(setpoint));
  }

  public void setPosition(double setpoint) {
    arm.setPosition(setpoint);
  }

  public Command setPositionBlocking(double setpoint) {
    this.setpoint = setpoint;
    return runEnd(() -> arm.setPosition(setpoint), () -> {})
        .until(() -> NemesisMathUtil.isApprox(inputs.armpos, setpointTolerance, setpoint));
  }

  public Command setPositionRun(double setpoint) {
    return run(() -> arm.setPosition(setpoint));
  }

  public Command setPositionLoggedTunableNumber() {
    return runEnd(() -> arm.setPositionLoggedNumber(), () -> arm.setPositionLoggedNumber());
  }

  public double getTunableNumber() {
    return arm.getTunableNumber();
  }

  public Command resetarm() {
    return runOnce(arm::resetArm);
  }

  public Command manual(DutyCycleOut request) {
    return runOnce(() -> arm.setPower(request));
  }

  public Command stop() {
    return runOnce(arm::stop);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void setVoltage(double volts) {
    arm.setVoltage(volts);
  }

  public double getAbsolutePosition() {
    return inputs.armpos;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public ArmIO getIO() {
    return arm;
  }

  public Rotation3d getArmRotation() {
    return new Rotation3d(0, -inputs.armpos / 0.25 * 90 * Math.PI / 180, 0);
  }
}
