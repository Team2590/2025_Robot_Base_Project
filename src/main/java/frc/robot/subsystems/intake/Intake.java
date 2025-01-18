package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert disconnected;
  private boolean isRunning = false;
  private double currentVoltage = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    disconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    disconnected.set(!inputs.connected);

    // Check beam break while intake is running
    if (isRunning && inputs.beamBreakTriggered) {
      io.stop();
      isRunning = false;
      Logger.recordOutput("Intake/StoppedByBeamBreak", true);
    }

//beam break detection
    Logger.recordOutput("Intake/BeamBreak", inputs.beamBreakTriggered);
    Logger.recordOutput("Intake/IsRunning", isRunning);
    Logger.recordOutput("Intake/CurrentVoltage", currentVoltage);
  }

  @AutoLogOutput
  public Command runIntake(double voltage) {
    return startEnd(
            () -> {
              System.out.println("Starting the intake command now!");
              if (!inputs.beamBreakTriggered) {
                io.setVoltage(voltage);
                isRunning = true;
                currentVoltage = voltage;
              }
            },
            () -> {
              System.out.println("Stopping the intake command now!");
              io.stop();
              isRunning = false;
              currentVoltage = 0.0;
            })
        .withName("Run Intake");
  }

//game piece check
  @AutoLogOutput
  public boolean hasGamePiece() {
    return inputs.beamBreakTriggered;
  }

//stop
  public void forceStop() {
    io.stop();
    isRunning = false;
    currentVoltage = 0.0;
  }
}