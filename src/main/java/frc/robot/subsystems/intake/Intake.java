package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
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

  private class IntakeArm extends SubsystemBase {
    private final Alert intakeArmDisconnected;
    private final IntakeArmIO intakeArmIO;
    private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();

    public IntakeArm(IntakeArmIO intakeArmIO) {
      this.intakeArmIO = intakeArmIO;
      Logger.processInputs("IntakeArm", intakeArmInputs);
      intakeArmDisconnected = new Alert("Intake Arm motor disconnected!", Alert.AlertType.kWarning);
    }

    public void periodic() {
      intakeArmIO.updateInputs(intakeArmInputs);
      intakeArmIO.updateTunableNumbers();
      intakeArmDisconnected.set(!intakeArmInputs.connected);
    }

    public Command setIntakeCoralPosition() {
      return runOnce(() -> intakeArmIO.setPosition(10));
    }

    public Command setIntakeAlgaePosition() {
      return runOnce(() -> intakeArmIO.setPosition(0));
    }

    public Command resetRotationCount() {
      return runOnce(() -> intakeArmIO.resetRotationCount());
    }

    public Command setPosition(double position) {
      return runOnce(() -> intakeArmIO.setPosition(position));
    }
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
    return intakeArm.setIntakeCoralPosition();
  }

  public Command setIntakeAlgaePosition() {
    return intakeArm.setIntakeAlgaePosition();
  }

  public Command setPosition(double position) {
    return intakeArm.setPosition(position);
  }

  public Command resetArmRotationCount() {
    return intakeArm.resetRotationCount();
  }
}
