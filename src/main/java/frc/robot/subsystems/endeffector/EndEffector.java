package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
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
  // private LoggedTunableNumber CURRENT_THRESHOLD =
  //     new LoggedTunableNumber("EndEffector/FilteredData", 200);
  private LoggedTunableNumber taps = new LoggedTunableNumber("EndEffector/taps", 10);
  private LinearFilter filter = LinearFilter.movingAverage((int) taps.get());
  private LoggedTunableNumber PROX_THRESHOLD =
      new LoggedTunableNumber("EndEffector/PROX_THRESHOLD", 500);
  double filtered_data;
  private LoggedTunableNumber runVoltage =
      new LoggedTunableNumber(
          "EndEffector/runVoltage", Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    filtered_data = filter.calculate(inputs.proxValue);

    Logger.recordOutput("EndEffector/StatorCurrent", inputs.statorCurrentAmps);
    Logger.recordOutput("EndEffector/IsRunning", isRunning);
    Logger.recordOutput("EndEffector/CurrentThreshold", CURRENT_THRESHOLD);
    Logger.recordOutput("EndEffector/filter", filtered_data);
    Logger.recordOutput("EndEffector/prox", inputs.proxValue);
  }

  public Command runEndEffector() {
    return runEnd(
            () -> {
              io.setVoltage(-Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);
            },
            () -> {
              io.stop();
            })
        .until(() -> hasCoral());
  }

  public Command runEndEffectorOuttake() {
    return runEnd(
            () -> {
              io.setVoltage(-Constants.EndEffectorConstantsLeonidas.RUN_VOLTAGE);
            },
            () -> {
              io.stop();
            })
        .until(() -> !hasCoral());
  }

  public Command runEndEffectorDeAlgae() {
    return runEnd(
        () -> {
          io.setVoltage(-Constants.EndEffectorConstantsLeonidas.DEALGAE_VOLTAGE);
        },
        () -> {
          io.stop();
        });
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

  /**
   * returns whether the end effector has a piece of coral or not
   *
   * @return true if the end effector has coral, false if not
   */
  public boolean hasCoral() {
    return filtered_data >= PROX_THRESHOLD.get();
  }

  public boolean isRunning() {
    return isRunning;
  }
}
