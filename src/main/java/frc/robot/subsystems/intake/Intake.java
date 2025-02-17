package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeArmIO intakeArmIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();
  private final Alert intakeDisconnected;
  private final IntakeArm intakeArm;

  public Intake(IntakeIO intakeIO, IntakeArmIO intakeArmIO) {
    this.intakeIO = intakeIO;
    this.intakeArmIO = intakeArmIO;
    intakeDisconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
    intakeArm = new IntakeArm(intakeArmIO);
    intakeIO.setNeutralMode(NeutralModeValue.Brake);
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
    private double setpointTolerance = 0.05;

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

    public Command resetRotationCount() {
      return runOnce(() -> intakeArmIO.resetRotationCount());
    }

    public Command setPosition(double position) {
      return runOnce(() -> intakeArmIO.setPosition(position));
    }

    public Command setPositionBlocking(double position) {
      return runEnd(
              () -> intakeArmIO.setPosition(position), () -> intakeArmIO.setPosition(position))
          .until(
              () ->
                  NemesisMathUtil.isApprox(
                      intakeArmInputs.positionRads, setpointTolerance, position));
    }
    public double getVelocityRadPerSec() {
      return intakeArmInputs.velocityRadsPerSec;
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

  public Command setPosition(double position) {
    return intakeArm.setPosition(position);
  }

  public Command setPositionBlocking(double position) {
    return intakeArm.setPositionBlocking(position);
  }

  public double getVelocityRadPerSec() {
    return intakeArmInputs.velocityRadsPerSec;
  }

  public Command resetArmRotationCount() {
    return intakeArm.resetRotationCount();
  }

  public void setVoltage(double volts) {
    intakeArmIO.setVoltage(volts);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return intakeArm.getVelocityRadPerSec();
  }
}
