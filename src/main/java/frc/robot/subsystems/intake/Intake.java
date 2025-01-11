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

  public Intake(IntakeIO io) {
    this.io = io;
    disconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    disconnected.set(!inputs.connected);
  }

  @AutoLogOutput
  public Command runIntake(double voltage) {
    return startEnd(
            () -> {
              System.out.println("Starting the intake command now!");
              io.setVoltage(voltage);
            },
            () -> {
              System.out.println("Stopping the intake command now!");
              io.stop();
            })
        .withName("Run Intake");
  }
}
