package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();
  private boolean isRunning = false;
  private LoggedTunableNumber PROX_THRESHOLD =
      new LoggedTunableNumber("EndEffector/proxThreshold", 200);
  private LoggedTunableNumber taps = new LoggedTunableNumber("EndEffector/taps", 10);
  private LinearFilter filter = LinearFilter.movingAverage((int) taps.get());
  double filtered_data;
  private LoggedTunableNumber runVoltage =
      new LoggedTunableNumber(
          "EndEffector/runVoltage", Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);
  private AnalogInput prox = new AnalogInput(Constants.EndEffectorConstantsLeonidas.PROX_CHANNEL);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    filtered_data = filter.calculate(prox.getValue());

    Logger.recordOutput("EndEffector/StatorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/IsRunning", isRunning);
    Logger.recordOutput("EndEffector/CurrentThreshold", PROX_THRESHOLD);
    Logger.recordOutput("EndEffector/filter", filtered_data);
    Logger.recordOutput("EndEffector/RawValue", prox.getValue());

    if (taps.hasChanged(0)) {
      filter = LinearFilter.movingAverage((int) taps.get());
    }
  }

  public Command runEndEffectorIntake() {
    return runEnd(
            () -> {
              io.setVoltage(Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);
            },
            () -> {
              io.stop();
            })
        .until(() -> hasGamePiece());
  }

  public Command runEndEffectorOuttake() {
    return runEnd(
            () -> {
              io.setVoltage(-Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);
            },
            () -> {
              io.stop();
            })
        .until(() -> !hasGamePiece());
  }

  public void runEndEffectorGrabAndHoldAlgae() {
    io.setVoltage(Constants.EndEffectorConstantsLeonidas.GRAB_ALGAE_VOLTAGE);
  }

  public Command runEndEffectorGrabAndHoldAlgaeCommand() {
    return Commands.run(() -> io.setVoltage(Constants.EndEffectorConstantsLeonidas.GRAB_ALGAE_VOLTAGE));
  }

  public Command runEndEffectorManual() {
    return runEnd(
        () -> {
          io.setVoltage(-runVoltage.get());
        },
        () -> {
          io.stop();
        });
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

  public Command runEndEffectorVoltage(double voltage) {
    return runEnd(
        () -> {
          io.setVelocity(voltage);
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

  public boolean hasGamePiece() {
    return filtered_data >= PROX_THRESHOLD.get();
  }

  public boolean isRunning() {
    return isRunning;
  }
}
