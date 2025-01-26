package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
    io.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Command resetRotationCount() {
    return runOnce(io::resetRotationCount);
  }

  public Command setNeutralMode(NeutralModeValue mode) {
    return runOnce(() -> io.setNeutralMode(mode));
  }

  public Command run(double voltage) {
    boolean motorOutOfBounds = Math.abs(inputs.rotationCount) > 153;

    return runEnd(() -> io.setVoltage(voltage), io::stop).until(() -> motorOutOfBounds);
  }

  public Command raise() {
    return runEnd(() -> io.setVoltage(3), io::stop);
  }

  public Command lower() {
    return runEnd(() -> io.setVoltage(-3), io::stop);
  }
}
