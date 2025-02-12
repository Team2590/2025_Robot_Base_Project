package frc.robot.command_factories;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

/**
 * Factory class for creating commands related to the elevator subsystem.
 *
 * <p>This class provides methods to create commands for controlling the elevator mechanism.
 * 
 */
public class ElevatorFactory {
  private static RobotContainer container = Robot.getRobotContainerInstance();

  /**
   * Creates a command to move the elevator to a specific position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param position The target position in rotation counts
   * @return Command to move elevator to position
   */
  public static Command setPosition(double position) {
    return container.getElevator().setPosition(position).withName("Set Elevator Position")
    .onlyIf(() -> NemesisMathUtil.isBetweenInclusive(
      container.getArm().getSetpoint(), 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS
    ));
  }

  /**
   * Creates a command to move the elevator to a specific position and wait until it reaches the
   * target.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param position The target position in rotation counts
   * @return Command to move elevator to position and wait
   */
  public static Command setPositionBlocking(double position) {
    return container
        .getElevator()
        .setPositionBlocking(position)
        .withName("Set Elevator Position Blocking");
  }

  /**
   * Creates a command to reset the elevator's rotation count.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @return Command to reset rotation count
   */
  public static Command resetRotationCount() {
    return container.getElevator().resetRotationCount().withName("Reset Elevator Rotation Count");
  }

  /**
   * Creates a command to set the elevator's neutral mode.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param mode The neutral mode to set
   * @return Command to set neutral mode
   */
  public static Command setNeutralMode(NeutralModeValue mode) {
    return container.getElevator().setNeutralMode(mode).withName("Set Elevator Neutral Mode");
  }

  /**
   * Creates a command to raise the elevator by one rotation count.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @return Command to raise elevator
   */
  public static Command raise() {
    return container.getElevator().raise().withName("Raise Elevator")
    .onlyIf(() -> NemesisMathUtil.isBetweenInclusive(
      container.getArm().getSetpoint(), 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS
    ));
  }

  /**
   * Creates a command to lower the elevator by one rotation count.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @return Command to lower elevator
   */
  public static Command lower() {
    return container.getElevator().lower().withName("Lower Elevator")
    .onlyIf(() -> NemesisMathUtil.isBetweenInclusive(
      container.getArm().getSetpoint(), 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS, 
      Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS
    ));
  }
}
