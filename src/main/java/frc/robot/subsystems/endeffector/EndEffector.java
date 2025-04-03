package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
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
      new LoggedTunableNumber("EndEffector/ProxThreshold", 1500);
  private LoggedTunableNumber CURRENT_THRESHOLD =
      new LoggedTunableNumber("EndEffector/CurrentThreshold", 15);
  private LoggedTunableNumber taps = new LoggedTunableNumber("EndEffector/taps", 10);
  private LinearFilter filter_current = LinearFilter.movingAverage((int) taps.get());
  private LinearFilter filter_prox = LinearFilter.movingAverage((int) taps.get());
  private double stator_current_filtered_data;
  private double prox_filtered_data;
  private LoggedTunableNumber runVoltage =
      new LoggedTunableNumber(
          "EndEffector/runVoltage", Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE);
  private AnalogInput prox = new AnalogInput(Constants.EndEffectorConstantsLeonidas.PROX_CHANNEL);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // filtered_data = filter.calculate(prox.getValue());
    // stator_current_filtered_data = filter_current.calculate(inputs.statorCurrentAmps);
    prox_filtered_data = filter_prox.calculate(prox.getValue());

    // Logger.recordOutput("EndEffector/current", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/proxValue", prox.getValue());

    if (taps.hasChanged(0)) {
      filter_current = LinearFilter.movingAverage((int) taps.get());
      filter_prox = LinearFilter.movingAverage((int) taps.get());
    }
  }

  public Command runEndEffectorIntake() {
    return runEnd(
            () -> {
              io.setVoltage(Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE);
            },
            () -> {
              io.stop();
            })
        .until(() -> hasGamePiece());
  }

  public Command runEndEffectorOuttake() {
    return runEnd(
            () -> {
              io.setVoltage(12);
            },
            () -> {
              io.stop();
            })
        .until(() -> !hasGamePiece());
  }

  public void runEndEffectorGrabAndHoldAlgae() {
    io.setVoltage(Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE);
  }

  public Command runEndEffectorGrabAndHoldAlgaeCommand() {
    return Commands.run(
        () -> io.setVoltage(-Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE));
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
          io.setVoltage(voltage);
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
    return
    // stator_current_filtered_data >= CURRENT_THRESHOLD.get();
    prox_filtered_data >= PROX_THRESHOLD.get();
  }

  public boolean isRunning() {
    return isRunning;
  }

  public EndEffectorIO getIO() {
    return io;
  }
}
