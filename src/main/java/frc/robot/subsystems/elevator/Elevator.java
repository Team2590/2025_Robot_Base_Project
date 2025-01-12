package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateTunableNumbers();
    io.updateInputs(inputs);
  }

  @AutoLogOutput
  public Command stop() {
    return Commands.runOnce(io::stop, this);
  }

  @AutoLogOutput
  public Command setPosition(double position) {
    return Commands.runOnce(() -> {
      io.setPosition(position);
    }, this);
  }

  @AutoLogOutput
  public Command resetRotationCount() {
    return Commands.runOnce(io::resetRotationCount, this);
  }

  @AutoLogOutput
  public Command raise() {
    return Commands.runOnce(() -> {
      io.setPosition(inputs.rotationCount + 05);
    }, this);
  }

  @AutoLogOutput
  public Command lower() {
    return Commands.runOnce(() -> {
      io.setPosition(inputs.rotationCount - 0.5);
    }, this);
  }
}
