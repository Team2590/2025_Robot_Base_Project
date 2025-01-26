package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final Alert intakeDisconnected;
  private final IntakeArm intakeArm;

  public Intake(IntakeIO intakeIO, IntakeArmIO intakeArmIO) {
    this.intakeIO = intakeIO;
    intakeDisconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
    intakeArm = new IntakeArm(intakeArmIO);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
    intakeDisconnected.set(!intakeInputs.connected);
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
    return Commands.runOnce(() -> intakeArm.setIntakeCoralPosition(), intakeArm)
        .withName("Set intake coral position");
  }

  public Command setIntakeAlgaePosition() {
    return Commands.runOnce(() -> intakeArm.setIntakeAlgaePosition(), intakeArm)
        .withName("Set intake algae position");
  }

  public Command resetArmRotationCount() {
    return Commands.runOnce(() -> intakeArm.resetRotationCount(), intakeArm)
        .withName("Set intake coral position");
  }
}
