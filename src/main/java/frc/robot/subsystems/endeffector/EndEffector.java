package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
  private boolean isRunning = false;
  private LoggedTunableNumber CURRENT_THRESHOLD =
      new LoggedTunableNumber("EndEffector/CURRENT_THRESHOLD", 200);
  private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double filtered_data = filter.calculate(inputs.statorCurrentAmps);
    // stopping
    if (isRunning && filtered_data >= 120 && inputs.statorCurrentAmps > filtered_data) {
      stopIntake().schedule();
      isRunning = false;
    }

    Logger.recordOutput("EndEffector/StatorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/IsRunning", isRunning);
    Logger.recordOutput("EndEffector/CurrentThreshold", CURRENT_THRESHOLD);
    Logger.recordOutput("EndEffector/filter", filtered_data);
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

  public Command runOuttake() {
    return runEnd(
            () -> {
              io.setVoltage(-6.0);
              isRunning = true;
            },
            () -> {
              io.stopMotor();
              isRunning = false;
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command runIntakeVelocity() {
    return runEnd(
        () -> {
          io.setVelocity(1);
          isRunning = true;
        },
        () -> {
          io.stopMotor();
          isRunning = false;
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          io.stopMotor();
          isRunning = false;
        });
  }
}
