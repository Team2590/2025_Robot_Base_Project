package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
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

  public void setIntakeCoralPosition() {
    intakeArmIO.setPosition(10);
  }

  public void setIntakeAlgaePosition() {
    intakeArmIO.setPosition(0);
  }

  public void resetRotationCount() {
    intakeArmIO.resetRotationCount();
  }
}
