package frc.robot.command_factories;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;
import frc.robot.util.SafetyChecker;

/**
 * Factory class for creating commands related to the elevator subsystem.
 *
 * <p>This class provides methods to create commands for controlling the elevator mechanism.
 */
public class ElevatorFactory {

  /**
   * Creates a command to move the elevator to a specific position.
   *
   * @param position The target position in rotation counts
   * @return Command to move elevator to position
   */
  public static Command setPosition(double position) {
    return RobotContainer.getElevator()
        .setPosition(position)
        .withName("Set Elevator Position")
        .onlyIf(
            () ->
                SafetyChecker.isSafe(
                    SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
                    position,
                    RobotContainer.getArm().getAbsolutePosition()));
  }

  /**
   * Creates a command to move the elevator to a specific position and wait until it reaches the
   * target.
   *
   * @param position The target position in rotation counts
   * @return Command to move elevator to position and wait
   */
  public static Command setPositionBlocking(double position) {
    return RobotContainer.getElevator()
        .setPositionBlocking(position)
        .withName("Set Elevator Position Blocking")
        .onlyIf(
            () ->
                SafetyChecker.isSafe(
                    SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
                    position,
                    RobotContainer.getArm().getAbsolutePosition()));
  }

  /**
   * Creates a command to reset the elevator's rotation count.
   *
   * @return Command to reset rotation count
   */
  public static Command resetRotationCount() {
    return RobotContainer.getElevator()
        .resetRotationCount()
        .withName("Reset Elevator Rotation Count");
  }

  /**
   * Creates a command to set the elevator's neutral mode.
   *
   * @param mode The neutral mode to set
   * @return Command to set neutral mode
   */
  public static Command setNeutralMode(NeutralModeValue mode) {
    return RobotContainer.getElevator().setNeutralMode(mode).withName("Set Elevator Neutral Mode");
  }

  /**
   * Creates a command to raise the elevator by one rotation count.
   *
   * @return Command to raise elevator
   */
  public static Command raise() {
    return RobotContainer.getElevator()
        .raise()
        .withName("Raise Elevator")
        .onlyIf(
            () ->
                NemesisMathUtil.isBetweenInclusive(
                    RobotContainer.getArm().getSetpoint(),
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS,
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS));
  }

  /**
   * Creates a command to lower the elevator by one rotation count.
   *
   * @return Command to lower elevator
   */
  public static Command lower() {
    return RobotContainer.getElevator()
        .lower()
        .withName("Lower Elevator")
        .onlyIf(
            () ->
                NemesisMathUtil.isBetweenInclusive(
                    RobotContainer.getArm().getSetpoint(),
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS,
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS));
  }
}
