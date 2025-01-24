package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final Alert intakeDisconnected;
  private final Alert intakeArmDisconnected;
  private final IntakeArmIO intakeElevatorIO;
  private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO, IntakeArmIO intakeArmIO) {
    this.intakeIO = intakeIO;
    this.intakeElevatorIO = intakeArmIO;
    intakeDisconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
    intakeArmDisconnected = new Alert("Intake Arm motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
    intakeElevatorIO.updateInputs(intakeArmInputs);
    intakeElevatorIO.updateTunableNumbers();
    Logger.processInputs("IntakeArm", intakeArmInputs);
    intakeDisconnected.set(!intakeInputs.connected);
    intakeArmDisconnected.set(!intakeArmInputs.connected);
  }

  public Command runIntake(double voltage) {
    return runEnd(
            () -> {
              System.out.println("Starting the intake command now!");
              intakeIO.setVoltage(voltage);
            },
            () -> {
              System.out.println("Stopping the intake command now!");
              intakeIO.stop();
            })
        .withName("Run Intake");
  }

  public Command setIntakeCoralPosition() {
    return runOnce(() -> intakeElevatorIO.setPosition(10));
  }

  public Command setIntakeAlgaePosition() {
    return runOnce(() -> intakeElevatorIO.setPosition(0));
  }

  public Command resetRotationCount() {
    return runOnce(() -> intakeElevatorIO.resetRotationCount());
  }
}
