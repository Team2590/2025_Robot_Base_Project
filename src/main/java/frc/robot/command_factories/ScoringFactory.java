package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.NemesisTimedCommand;

/**
 * Factory class for creating complex scoring-related commands.
 *
 * <p>This class provides methods to create composite commands that coordinate multiple subsystems
 * for scoring operations.
 */
public class ScoringFactory {

  public enum Level {
    L1(0 /* Not Using Elevator at L1 */),
    L2(Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS),
    L3(Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS),
    L4(Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS);

    private final double elevatorPosition;

    private Level(double elevatorPosition) {
      this.elevatorPosition = elevatorPosition;
    }

    public double getElevatorPosition() {
      return elevatorPosition;
    }
  }

  /**
   * Creates a command sequence for scoring at L4.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L4 scoring
   */
  public static Command scoreL4() {
    return new ParallelCommandGroup(
            ElevatorFactory.setPositionBlocking(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4)
            // NemesisTimedCommand.generateTimedCommand(EndEffectorFactory.runEndEffectorOuttake(),
            // 1)
            )
        .withName("Score L4");
  }

  public static Command score(Level level) {
    return switch (level) {
      case L1:
        yield scoreL1();
      default:
        yield primeForLevel(level)
            .andThen(EndEffectorFactory.runEndEffectorOuttake())
            .until(() -> !RobotState.getInstance().endEffectorhasCoral())
            .withName("Score " + level.name());
    };
  }

  public static Command primeForLevel(Level level) {
    return Commands.parallel(
            Commands.print("Priming " + level.name()),
            ElevatorFactory.setPositionBlocking(level.getElevatorPosition()),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS))
        .withName("Prime " + level.name());
  }

  /**
   * Creates a command sequence for scoring at L1.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L1 scoring
   */
  public static Command scoreL1() {
    return Commands.parallel(
        IntakeFactory.setHomePosition(),
        NemesisTimedCommand.generateTimedCommand(
                IntakeFactory.runIntake(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED),
                1)
            .withName("Score L1"));
  }

  /**
   * Creates a command sequence for scoring at processor.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for processor scoring
   */
  public static Command scoreProcessor() {
    return Commands.parallel(
        IntakeFactory.setHoldingAlgaePosition(),
        NemesisTimedCommand.generateTimedCommand(
                IntakeFactory.runIntake(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_ALGAE_OUTTAKE_SPEED),
                1)
            .withName("Score Processor"));
  }

  /**
   * Creates a command to stow the scoring mechanism.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for stowing
   */
  public static Command stow() {
    return Commands.sequence(
            ArmFactory.setPositionBlocking(0), ElevatorFactory.setPositionBlocking(0))
        .withName("Stow Mechanism");
  }

  public static Command prepClimb() {
    // return new ParallelCommandGroup(
    // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS),
    // ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
    return new ParallelCommandGroup(
            IntakeFactory.setIntakeCoralPosition(),
            ArmFactory.setPositionBlocking(.33),
            ElevatorFactory.setPositionBlocking(0.5))
        .withName("Deploy climb mechanism");
    // , ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION)
  }

  public static Command deployMechanism() {
    return ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION);
  }

  public static Command climb() {
    return Commands.parallel(
            // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS),
            // ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
            ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION))
        .withName("Climb");
  }
}
