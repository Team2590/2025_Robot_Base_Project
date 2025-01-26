package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO arm;

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    arm = io;
  }

  @Override
  public void periodic() {
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  /** Run open loop at the specified voltage. */
  public Command setPosition(double setpoint) {
    return runOnce(() -> arm.setPosition(setpoint));
  }

  public Command resetarm() {
    return runOnce(arm::resetArm);
  }

  public Command manual(DutyCycleOut request) {
    return runOnce(() -> arm.setPower(request));
  }

  /** Stops the flywheel. */
  public Command stop() {
    return runOnce(arm::stop);
  }
}
