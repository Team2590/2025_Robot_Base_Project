package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * Factory class for creating complex scoring-related commands.
 *
 * <p>This class provides methods to create composite commands that coordinate multiple subsystems
 * for scoring operations.
 */
public class ScoringFactory {
  /**
   * Creates a command sequence for scoring at a high position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for high scoring
   */
  public static Command scoreHigh() {
    return new SequentialCommandGroup(
            // First, move elevator to high position
            ElevatorFactory.setPosition(48),

            // Then, extend arm to scoring position
            ArmFactory.setPosition(90),

            // Finally, run intake to release game piece
            IntakeFactory.runIntake(() -> -6))
        .withName("Score High");
  }

  /**
   * Creates a command sequence for scoring at a mid position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for mid scoring
   */
  public static Command scoreMid() {
    return new SequentialCommandGroup(
            // First, move elevator to mid position
            ElevatorFactory.setPosition(30),

            // Then, extend arm to scoring position
            ArmFactory.setPosition(60),

            // Finally, run intake to release game piece
            IntakeFactory.runIntake(() -> -6))
        .withName("Score Mid");
  }

  /**
   * Creates a command to prepare for scoring by moving both elevator and arm simultaneously.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @param elevatorPosition Target elevator position
   * @param armPosition Target arm position
   * @return Parallel command for scoring preparation
   */
  public static Command prepareForScoring(
      RobotContainer container, double elevatorPosition, double armPosition) {
    return new ParallelCommandGroup(
            ElevatorFactory.setPosition(elevatorPosition),
            ArmFactory.setPosition(armPosition))
        .withName("Prepare For Scoring");
  }

  /**
   * Creates a command to stow the scoring mechanism.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for stowing
   */
  public static Command stow(RobotContainer container) {
    return new SequentialCommandGroup(
            // First, retract arm
            ArmFactory.setPosition(0),

            // Then, lower elevator
            ElevatorFactory.setPosition(0))
        .withName("Stow Mechanism");
  }
}
