package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeArmIO intakeArmIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  // private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();
  private final Alert intakeDisconnected;
  private final IntakeArm intakeArm;
  private LoggedTunableNumber RUNNING_THRESHOLD =
      new LoggedTunableNumber("Intake/LOWER_THRESHOLD", 0.25);
  private LoggedTunableNumber HOLDING_THRESHOLD =
      new LoggedTunableNumber("Intake/HIGHER_THRESHOLD", 0.5);
      private LoggedTunableNumber INTAKE_CORAL_CURRENT_THRESHOLD =
      new LoggedTunableNumber("Intake/CoralCurrentThreshold", 0.5);
  private LinearFilter filter = LinearFilter.movingAverage(30);
  double filtered_data;

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
    filtered_data = filter.calculate(intakeInputs.torqueCurrentAmps);
    Logger.recordOutput("Intake/filter", filtered_data);
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

    public Command setIntakeCoralPosition() {
      return runOnce(() -> intakeArmIO.setPosition(10));
    }

    public Command setIntakeAlgaePosition() {
      return runOnce(() -> intakeArmIO.setPosition(4));
    }

    public Command resetRotationCountCommand() {
      return runOnce(() -> intakeArmIO.resetRotationCount());
    }

    public void resetRotationCount() {
      intakeArmIO.resetRotationCount();
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

  public Command resetArmRotationCountCommand() {
    return intakeArm.resetRotationCountCommand();
  }

  public void resetArmRotationCount() {
    intakeArm.resetRotationCount();
  }

  public void setVoltage(double volts) {
    intakeArmIO.setVoltage(volts);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return intakeArm.getVelocityRadPerSec();
  }

  /**
   * Returns boolean whether the intake has the algae (not running)
   *
   * @return true if the intake has secured the algae, false if not
   */
  public boolean hasAlgae() {
    return filtered_data >= HOLDING_THRESHOLD.get() || filtered_data <= RUNNING_THRESHOLD.get();
  }

  /**
   * Returns boolean whether the intake is running or not
   *
   * @return true if intake is running, false if not
   */
  public boolean isRunning() {
    return filtered_data >= HOLDING_THRESHOLD.get() && filtered_data >= RUNNING_THRESHOLD.get();
  }

  /*
   * 
   * The way we can distinguish between Algae and Coral is by using the sign of the current
   *  TODO figure out the direction of intake coral vs algae
   */
  public boolean hasCoral(){
    return filtered_data >= INTAKE_CORAL_CURRENT_THRESHOLD.get();
  }
}
