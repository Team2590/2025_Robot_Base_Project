package frc.robot.command_factories;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Factory class for creating commands related to the elevator subsystem.
 *
 * <p>This class provides methods to create commands for controlling the elevator mechanism.
 */
public class ElevatorFactory {
  /**
   * Creates a command to move the elevator to a specific position.
   *
   * @param container The RobotContainer instance
   * @param position The target position in rotation counts
   * @return Command to move elevator to position
   */
  public static Command setPosition(RobotContainer container, double position) {
    return container.getElevator().setPosition(position).withName("Set Elevator Position");
  }

  /**
   * Creates a command to move the elevator to a specific position and wait until it reaches the
   * target.
   *
   * @param container The RobotContainer instance
   * @param position The target position in rotation counts
   * @return Command to move elevator to position and wait
   */
  public static Command setPositionBlocking(RobotContainer container, double position) {
    return container
        .getElevator()
        .setPositionBlocking(position)
        .withName("Set Elevator Position Blocking");
  }

  /**
   * Creates a command to reset the elevator's rotation count.
   *
   * @param container The RobotContainer instance
   * @return Command to reset rotation count
   */
  public static Command resetRotationCount(RobotContainer container) {
    return container.getElevator().resetRotationCount().withName("Reset Elevator Rotation Count");
  }

  /**
   * Creates a command to set the elevator's neutral mode.
   *
   * @param container The RobotContainer instance
   * @param mode The neutral mode to set
   * @return Command to set neutral mode
   */
  public static Command setNeutralMode(RobotContainer container, NeutralModeValue mode) {
    return container.getElevator().setNeutralMode(mode).withName("Set Elevator Neutral Mode");
  }

  /**
   * Creates a command to raise the elevator by one rotation count.
   *
   * @param container The RobotContainer instance
   * @return Command to raise elevator
   */
  public static Command raise(RobotContainer container) {
    return container.getElevator().raise().withName("Raise Elevator");
  }

  /**
   * Creates a command to lower the elevator by one rotation count.
   *
   * @param container The RobotContainer instance
   * @return Command to lower elevator
   */
  public static Command lower(RobotContainer container) {
    return container.getElevator().lower().withName("Lower Elevator");
  }
}
