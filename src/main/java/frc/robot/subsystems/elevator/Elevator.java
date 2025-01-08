package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io = new ElevatorIOTalonFX();
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateTunableNumbers();
    io.updateInputs(inputs);
  }

  public void setPosition(double position) {
    if (inputs.absRotationCount <= 0) {
      io.stop();
    } else {
      io.setPosition(position);
    }
  }

  public void raise() {
    if (inputs.absRotationCount <= 0) {
      io.stop();
    } else {
      setPosition(inputs.absRotationCount + 0.5);
    }
  }

  public void lower() {
    if (inputs.absRotationCount <= 0) {
      io.stop();
    } else {
      setPosition(inputs.absRotationCount - 0.5);
    }
  }

  public void stop() {
    io.stop();
  }
}
