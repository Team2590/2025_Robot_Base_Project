package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
  private boolean isRunning = false;

  private final double CURRENT_THRESHOLD = 75.0;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // stopping
    if (isRunning && inputs.statorCurrentAmps >= CURRENT_THRESHOLD) {
      stopIntake().schedule();
      isRunning = false;
    }

    Logger.recordOutput("EndEffector/StatorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/IsRunning", isRunning);
    Logger.recordOutput("EndEffector/CurrentThreshold", CURRENT_THRESHOLD);
  }

  public Command runIntake() {
    return runEnd(
            () -> {
              io.setVoltage(6.0);
              isRunning = true;
            },
            () -> {
              io.stopMotor();
              isRunning = false;
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          io.stopMotor();
          isRunning = false;
        });
  }
}
