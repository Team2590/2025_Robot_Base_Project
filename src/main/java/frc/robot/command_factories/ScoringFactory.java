package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.NemesisTimedCommand;

/**
 * Factory class for creating complex scoring-related commands.
 *
 * <p>This class provides methods to create composite commands that coordinate multiple subsystems
 * for scoring operations.
 */
public class ScoringFactory {
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
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_L4_POS)
            // NemesisTimedCommand.generateTimedCommand(EndEffectorFactory.runEndEffectorOuttake(),
            // 1)
            )
        .withName("Score L4");
  }

  /**
   * Creates a command sequence for scoring at L3.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L3 scoring
   */
  public static Command scoreL3() {
    return new ParallelCommandGroup(
            ElevatorFactory.setPositionBlocking(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_L3_POS)
            // NemesisTimedCommand.generateTimedCommand(EndEffectorFactory.runEndEffectorOuttake(),
            // 1)
            )
        .withName("Score L3");
  }

  /**
   * Creates a command sequence for scoring at L2.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L2 scoring
   */
  public static Command scoreL2() {
    return new ParallelCommandGroup(
            ElevatorFactory.setPositionBlocking(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_L2_POS)
            // NemesisTimedCommand.generateTimedCommand(EndEffectorFactory.runEndEffectorOuttake(),
            // 1)
            )
        .withName("Score L2");
  }

  /**
   * Creates a command sequence for scoring at L1.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L1 scoring
   */
  public static Command scoreL1() {
    return new ParallelCommandGroup(
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
    return new ParallelCommandGroup(
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
  public static Command stow(RobotContainer container) {
    return new SequentialCommandGroup(
            ArmFactory.setPositionBlocking(0), ElevatorFactory.setPositionBlocking(0))
        .withName("Stow Mechanism");
  }

  public static Command deployClimbMechanism() {
    return new ParallelCommandGroup(
        ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS), 
        ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
        ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION)
    )
    .withName("Deploy climb mechanism");
  }

  public static Command climb() {
    return new ParallelCommandGroup(
        ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS), 
        ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
        ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION)
    )
    .withName("Climb");
  }
}
