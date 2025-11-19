package frc.robot.subsystems.peashooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PeaShooterConstants;
import org.littletonrobotics.junction.Logger;

public class PeaShooter extends SubsystemBase {
  private final PeaShooterIO io;
  private final PeaShooterIOInputsAutoLogged inputs = new PeaShooterIOInputsAutoLogged();

  public PeaShooter(PeaShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PeaShooter", inputs);
  }

  /** Run the shooter at the specified velocity. */
  public Command runShooter(double velocityRadPerSec) {
    return run(() -> {
          io.setLeftVelocity(velocityRadPerSec);
          io.setRightVelocity(velocityRadPerSec);
        })
        .finallyDo(
            () -> {
              io.stop();
            });
  }

  /** Run the shooter at a set voltage (open loop). */
  public Command runShooterVoltage(double volts) {
    return run(() -> {
          io.setLeftVoltage(volts);
          io.setRightVoltage(volts);
        })
        .finallyDo(
            () -> {
              io.stop();
            });
  }

  /** Set the hood position. */
  public Command setHoodPosition(double degrees) {
    return run(() -> {
      double radians = Units.degreesToRadians(degrees);
      // Clamp to min/max
      double clampedRadians =
          Math.max(
              Units.degreesToRadians(PeaShooterConstants.HOOD_MIN_POS),
              Math.min(Units.degreesToRadians(PeaShooterConstants.HOOD_MAX_POS), radians));
      io.setHoodPosition(clampedRadians);
    });
  }

  /** Stop the shooter. */
  public Command stop() {
    return runOnce(
        () -> {
          io.stop();
        });
  }
}
