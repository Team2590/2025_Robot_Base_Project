package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command runClimb(double voltage) {
    return runEnd(() -> io.setVoltage(voltage), io::stop);
  }

  public Command resetRotationCount() {
    return runOnce(io::resetRotationCount);
  }

  public double getRotationCount() {
    return inputs.rotationCount;
  }

  public void resetRotationCountFunction() {
    io.resetRotationCount();
  }
}
