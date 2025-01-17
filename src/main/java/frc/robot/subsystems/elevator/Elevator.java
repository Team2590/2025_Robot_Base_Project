package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

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
    return runOnce(io::stop);
  }

  @AutoLogOutput
  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position));
  }

  @AutoLogOutput
  public Command resetRotationCount() {
    return runOnce(io::resetRotationCount);
  }

  @AutoLogOutput
  public Command setNeutralMode(NeutralModeValue mode) {
    return runOnce(() -> io.setNeutralMode(mode));
  }

  @AutoLogOutput
  public Command raise() {
    return runOnce(() -> io.setPosition(inputs.rotationCount + 1));
  }

  @AutoLogOutput
  public Command lower() {
    return runOnce(() -> io.setPosition(inputs.rotationCount - 1));
  }
}
