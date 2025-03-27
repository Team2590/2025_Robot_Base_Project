package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double setpointTolerance = 0.05;
  private LoggedTunableNumber offset =
      new LoggedTunableNumber("Elevator/PosOFFSET", Constants.ElevatorConstantsLeonidas.OFFSET);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateTunableNumbers();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Log current position and target position
    Logger.recordOutput("Elevator/CurrentPosition", inputs.rotationCount);
    Logger.recordOutput("Elevator/TargetPosition", this.io.getTargetPosition());
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position + offset.get()));
  }

  public Command setPositionBlocking(double position) {
    return runEnd(
            () -> io.setPosition(position + offset.get()),
            () -> io.setPosition(position + offset.get()))
        .until(
            () ->
                NemesisMathUtil.isApprox(
                    inputs.rotationCount, setpointTolerance, position + offset.get()));
  }

  public Command setPositionRun(double setpoint) {
    return run(() -> io.setPosition(setpoint));
  }

  public Command setPositionLoggedTunableNumber() {
    return runEnd(() -> io.setPositionLoggedNumber(), () -> io.setPositionLoggedNumber());
  }

  public double getTunableNumber() {
    return io.getTunableNumber();
  }

  public Command resetRotationCountCommand() {
    return runOnce(io::resetRotationCount);
  }

  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public Command setNeutralMode(NeutralModeValue mode) {
    return runOnce(() -> io.setNeutralMode(mode));
  }

  public Command raise() {
    return runEnd(
        () -> io.setPosition(inputs.rotationCount + 1), () -> io.setPosition(inputs.rotationCount));
  }

  public Command lower() {
    return runEnd(
        () -> io.setPosition(inputs.rotationCount - 1), () -> io.setPosition(inputs.rotationCount));
  }

  public void setVoltage(double volts) {
    io.setVoltage(new VoltageOut(volts));
  }

  public Command setVoltageCommand(double volts) {
    return runEnd(
        () -> io.setVoltage(new VoltageOut(volts)), () -> io.setPosition(inputs.rotationCount));
  }
  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public double getRotationCount() {
    return inputs.rotationCount;
  }

  public ElevatorIO getIO() {
    return io;
  }
}
