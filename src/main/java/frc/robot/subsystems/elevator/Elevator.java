package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io = new ElevatorIOTalonFX();
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double position = 0;
  private boolean isRunning = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateTunableNumbers();
    io.updateInputs(inputs);
    io.setPosition(position);

    if (isRunning && inputs.absRotationCount != position) {
      io.setPosition(position);
      isRunning = true;
    }
  }

  public void setPosition(double position) {
    if (inputs.rotationCount <= 0) {
      io.stop();
      isRunning = false;
    } else {
      this.position = position;
      isRunning = true;
    }
  }

  public void raise() {
    isRunning = true;
    setPosition(inputs.absRotationCount + 0.5);
  }

  public void lower() {
    isRunning = true;
    setPosition(inputs.absRotationCount - 0.5);
  }

  public void stop() {
    io.stop();
    isRunning = false;
  }
}
