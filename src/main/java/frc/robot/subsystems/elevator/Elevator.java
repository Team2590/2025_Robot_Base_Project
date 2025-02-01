package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double setpointTolerance = 0.05;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateTunableNumbers();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position));
  }

  public Command setPositionBlocking(double position) {
    return runEnd(() -> io.setPosition(position), () -> io.setPosition(position))
        .until(() -> NemesisMathUtil.isApprox(inputs.leaderRotationCount, setpointTolerance, position));
  }

  public Command resetRotationCount() {
    return runOnce(io::resetRotationCount);
  }

  public Command setNeutralMode(NeutralModeValue mode) {
    return runOnce(() -> io.setNeutralMode(mode));
  }

  public Command raise() {
    return runEnd(() -> io.setPosition(inputs.leaderRotationCount + 1), io::stop);
  }

  public Command lower() {
    return runEnd(() -> io.setPosition(inputs.leaderRotationCount - 1), io::stop);
  }
}
