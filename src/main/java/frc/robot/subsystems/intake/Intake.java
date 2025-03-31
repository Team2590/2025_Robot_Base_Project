package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// spotless:off
public class Intake extends SubsystemBase {
  public IntakeIO intakeIO;
  private IntakeArmIO intakeArmIO;
  private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  // private IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();
  // private Alert intakeDisconnected;
  private IntakeArm intakeArm;
  private LoggedTunableNumber PROX_ONE_THRESHOLD = new LoggedTunableNumber("Intake/ProxOneThreshold", 500);
  private LoggedTunableNumber PROX_TWO_THRESHOLD = new LoggedTunableNumber("Intake/ProxTwoThreshold", 500);
  private LoggedTunableNumber LINEAR_FILTER_SAMPLES = new LoggedTunableNumber("Intake/LinearFilterSamples", 10);
  private LinearFilter proxOneFilter;
  private LinearFilter proxTwoFilter;
  private double proxOneFilteredData;
  private double proxTwoFilteredData;
  private AnalogInput proxOne = new AnalogInput(Constants.IntakeConstantsLeonidas.PROX_ONE_CHANNEL);
  private AnalogInput proxTwo = new AnalogInput(Constants.IntakeConstantsLeonidas.PROX_TWO_CHANNEL);

  public Intake(IntakeIO intakeIO, IntakeArmIO intakeArmIO) {
    this.intakeIO = intakeIO;
    this.intakeArmIO = intakeArmIO;
    // intakeDisconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
    intakeArm = new IntakeArm(intakeArmIO);
    intakeIO.setNeutralMode(NeutralModeValue.Brake);
    proxOneFilter = LinearFilter.movingAverage((int) LINEAR_FILTER_SAMPLES.get());
    proxTwoFilter = LinearFilter.movingAverage((int) LINEAR_FILTER_SAMPLES.get());
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
    // intakeDisconnected.set(!intakeInputs.connected);

    proxOneFilteredData = proxOneFilter.calculate(proxOne.getValue());
    proxTwoFilteredData = proxTwoFilter.calculate(proxTwo.getValue());

    Logger.recordOutput("Intake/ProxOneFiltered", proxOneFilteredData);
    Logger.recordOutput("Intake/ProxTwoFiltered", proxTwoFilteredData);

    if (LINEAR_FILTER_SAMPLES.hasChanged(0)) {
      proxOneFilter = LinearFilter.movingAverage((int) LINEAR_FILTER_SAMPLES.get());
      proxTwoFilter = LinearFilter.movingAverage((int) LINEAR_FILTER_SAMPLES.get());
    }
  }

  private class IntakeArm extends SubsystemBase {
    // private final Alert intakeArmDisconnected;
    private final IntakeArmIO intakeArmIO;
    private final IntakeArmIOInputsAutoLogged intakeArmInputs = new IntakeArmIOInputsAutoLogged();
    private double setpointTolerance = 0.05;

    public IntakeArm(IntakeArmIO intakeArmIO) {
      this.intakeArmIO = intakeArmIO;
      Logger.processInputs("IntakeArm", intakeArmInputs);
      // intakeArmDisconnected = new Alert("Intake Arm motor disconnected!", Alert.AlertType.kWarning);
    }

    public void periodic() {
      intakeArmIO.updateInputs(intakeArmInputs);
      intakeArmIO.updateTunableNumbers();
      // intakeArmDisconnected.set(!intakeArmInputs.connected);
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
      return runEnd(() -> intakeArmIO.setPosition(position), () -> intakeArmIO.setPosition(position))
          .until(() -> {
              System.out.println("input position rads:" + intakeArmInputs.positionRads);
              System.out.println("setpoint" + Units.rotationsToRadians(position));
              return NemesisMathUtil.isApprox(intakeArmInputs.rotationCount, setpointTolerance, position);
          });
    }

    public double getVelocityRadPerSec() {
      return intakeArmInputs.velocityRadsPerSec;
    }

    public double getRotationCount() {
      return intakeArmInputs.rotationCount;
    }
  }

  public Command runIntakeUntilHasCoral(double voltage) {
    return runEnd(
            () -> {
              System.out.println("Starting the intake command now!");
              intakeIO.setVoltage(voltage);
            },
            () -> {
              System.out.println("Stopping the intake command now!");
              intakeIO.stop();
            })
        .until(() -> hasCoral())
        .withName("Run Intake");
  }

  public Command runIntakeVoltage(double voltage) {
    return runEnd(() -> intakeIO.setVoltage(voltage), intakeIO::stop)
        .withName("Run Intake Voltage");
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

  public double getArmRotationCount() {
    return intakeArm.getRotationCount();
  }

  public IntakeArmIO getArmIO() {
    return intakeArmIO;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return proxOneFilteredData > PROX_ONE_THRESHOLD.get() && proxTwoFilteredData > PROX_TWO_THRESHOLD.get();
  }

  @AutoLogOutput
  public boolean detectCoral() {
    return proxOneFilteredData > PROX_ONE_THRESHOLD.get() || proxTwoFilteredData > PROX_TWO_THRESHOLD.get();
  }
}
// spotless:on
