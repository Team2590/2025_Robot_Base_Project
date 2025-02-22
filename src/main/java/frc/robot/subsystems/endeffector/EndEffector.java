package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
  private boolean isRunning = false;
  private LoggedTunableNumber CURRENT_THRESHOLD =
      new LoggedTunableNumber("EndEffector/CURRENT_THRESHOLD", 200);
  private LinearFilter filter = LinearFilter.movingAverage(30);
  double filtered_data;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    filtered_data = filter.calculate(inputs.statorCurrentAmps);

    Logger.recordOutput("EndEffector/StatorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/IsRunning", isRunning);
    Logger.recordOutput("EndEffector/CurrentThreshold", CURRENT_THRESHOLD);
    Logger.recordOutput("EndEffector/filter", filtered_data);
  }
  // This method is to check if we have a coral or not, if we don't have a coral the current will
  // not spike
  public boolean currentSpiked() {
    Timer t = new Timer();

    return t.hasElapsed(.2) && isRunning && filtered_data >= CURRENT_THRESHOLD.get();
  }

  public Command runEndEffector() {
    return runEnd(
            () -> {
              io.setVoltage(Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE);
              isRunning = true;
            },
            () -> {
              io.stop();
              isRunning = false;
            })
        .until(() -> filtered_data >= CURRENT_THRESHOLD.get())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command runEndEffectorOuttake() {
    return runEnd(
            () -> {
              io.setVoltage(Constants.EndEffectorConstantsLeonidas.EJECT_VOLTAGE);
              isRunning = true;
            },
            () -> {
              io.stop();
              isRunning = false;
            })
        .until(() -> isRunning && filtered_data >= CURRENT_THRESHOLD.get())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command runEndEffectorVelocity() {
    return runEnd(
        () -> {
          io.setVelocity(1);
          isRunning = true;
        },
        () -> {
          io.stop();
          isRunning = false;
        });
  }

  public Command stopEndEffector() {
    return runOnce(
        () -> {
          io.stop();
          isRunning = false;
        });
  }

  public boolean isRunning() {
    return isRunning;
  }
}
