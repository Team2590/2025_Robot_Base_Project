package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.HelperFn;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  protected final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert disconnected;
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private ArmStates state;
  private double armSetpoint;
  private double tolerance = .005;
  private DutyCycleOut power = new DutyCycleOut(0);
  private LoggedTunableNumber homeSetpoint = new LoggedTunableNumber("Arm/homeSetpoint", .168);
  private double targetHome = ArmConstants.HOME_SETPOINT;
  private boolean isClimbing = false;

  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  public static enum ArmStates {
    STOPPED,
    MANUAL,
    AT_SETPOINT,
    APPROACHINGSETPOINT,
    REEF, /*STOWED, */
    APPROACHING_HOME,
    HOME,
  }

  public Intake(IntakeIO io) {
    this.io = io;
    disconnected = new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);
    state = ArmStates.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    disconnected.set(!inputs.connected);
    arm.updateTunableNumbers();
    arm.updateInputs(armInputs);
    Logger.processInputs("Arm", inputs);
    switch (state) {
      case STOPPED:
        arm.stop();
        break;
      case MANUAL:
        arm.setPower(power);
        break;
      case APPROACHINGSETPOINT:
        arm.setPosition(armSetpoint);
        if (!isArmAtSetPointPosition(armSetpoint)) {
          state = ArmStates.APPROACHINGSETPOINT;
        } else {
          state = ArmStates.AT_SETPOINT;
        }
        break;
      case AT_SETPOINT:
        if (isArmAtSetPointPosition(armSetpoint)) {
          state = ArmStates.AT_SETPOINT;
          if (!isClimbing && armSetpoint == ArmConstants.HOME_SETPOINT) {
            state = ArmStates.HOME;
          }
        } else {
          state = ArmStates.APPROACHINGSETPOINT;
        }
        break;
      case REEF:
        break;
      case APPROACHING_HOME:
        arm.setPosition(ArmConstants.HOME_SETPOINT);
        if (isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
          state = ArmStates.HOME;
        }
      case HOME:
        if (!isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
          state = ArmStates.APPROACHING_HOME;
        }
        break;
    }
  }

  private boolean isArmAtSetPointPosition(double setPoint) {
    return HelperFn.isWithinTolerance(
        arm.armCancoder.getAbsolutePosition().getValueAsDouble(), setPoint, tolerance);
  }

  /** Run open loop at the specified voltage. */
  public void setPosition(double setpoint) {
    if (setpoint <= ArmConstants.ARM_MAX) {
      setpoint = ArmConstants.ARM_MAX;
    } else if (setpoint >= ArmConstants.CLIMB_SETPOINT) {
      setpoint = ArmConstants.CLIMB_SETPOINT;
    }
    armSetpoint = setpoint;
    if (!HelperFn.isWithinTolerance(armInputs.armabspos, armSetpoint, tolerance)) {
      state = ArmStates.APPROACHINGSETPOINT;
    } else {
      state = ArmStates.AT_SETPOINT;
    }
  }

  public void setHome() {
    if (isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
      state = ArmStates.HOME;
    } else {
      state = ArmStates.APPROACHING_HOME;
    }
  }

  public void setClimb() {
    isClimbing = true;
    if (!isArmAtSetPointPosition(ArmConstants.CLIMB_SETPOINT)) {
      setPosition(ArmConstants.CLIMB_SETPOINT);
    }
  }

  public void resetarm() {
    isClimbing = false;
    arm.resetArm();
  }

  public void manual(DutyCycleOut request) {
    power = request;
    state = ArmStates.MANUAL;
  }

  public void armmanualdown() {
    state = ArmStates.MANUAL;
    DutyCycleOut power = new DutyCycleOut(0.1);
    arm.setPower(power);
  }

  /** Stops the flywheel. */
  public void setStopped() {
    state = ArmStates.STOPPED;
  }

  public double getAbsolutePosition() {
    return arm.armCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public ArmStates getState() {
    return state;
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
