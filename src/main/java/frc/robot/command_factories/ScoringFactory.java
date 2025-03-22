package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.util.NemesisMathUtil;
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
    L4(Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS),
    SOURCE(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS);

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
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4))
        .withName("Score L4");
  }

  public static Command score(Level level) {
    return switch (level) {
      case L1:
        yield scoreL1();
      default:
        yield primeForLevel(level)
            .andThen(EndEffectorFactory.runEndEffectorOuttake())
            .until(() -> !RobotState.endEffectorhasCoral())
            .withName("Score " + level.name());
    };
  }

  public static Command primeForLevel(Level level) {
    switch (level) {
      case L4:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            ElevatorFactory.setPositionBlocking(level.getElevatorPosition()),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4)
                .withName("Prime " + level.name()));
      case L3:
        Commands.parallel(
            Commands.print("Priming " + level.name()),
            ElevatorFactory.setPositionBlocking(level.getElevatorPosition()),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3)
                .withName("Prime " + level.name()));
      default:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            ElevatorFactory.setPositionBlocking(level.getElevatorPosition()),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS)
                .withName("Prime " + level.name()));
    }
  }

  public static Command scoreTeleop(Level level) {
    return switch (level) {
      case L1:
        yield scoreL1();
      default:
        yield primeForLevelTeleop(level).withName("Score " + level.name());
    };
  }

  public static Command primeForLevelTeleop(Level level) {
    switch (level) {
      case L4:
        return Commands.sequence(
                Commands.parallel(
                    Commands.print("Priming " + level.name()),
                    ElevatorFactory.setPositionRun(level.getElevatorPosition())),
                ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4))
            .withName("Prime " + level.name());
      case L3:
        return Commands.sequence(
                Commands.parallel(
                    Commands.print("Priming " + level.name()),
                    ElevatorFactory.setPositionRun(level.getElevatorPosition())),
                ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3))
            .withName("Prime " + level.name());
      default:
        return Commands.sequence(
                Commands.parallel(
                    Commands.print("Priming " + level.name()),
                    ElevatorFactory.setPositionRun(level.getElevatorPosition())),
                ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS))
            .withName("Prime " + level.name());
    }
  }

  /**
   * Creates a command sequence for scoring at L1.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for L1 scoring
   */
  public static Command scoreL1() {
    return Commands.sequence(
        IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.L1_POS),
        IntakeFactory.runIntake(() -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED)
            .withName("Score L1"));
  }

  // public static Command deAlgaeify() {
  //   return Commands.sequence(ElevatorFactory.setPositionBlocking())
  // }

  public static Command scoreAlgaeBarge() {
    return ElevatorFactory.setPositionRun(Constants.ElevatorConstantsLeonidas.ELEVATOR_BARGE_POS)
    .alongWith(ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_BARGE_POS))
    .andThen(EndEffectorFactory.runEndEffectorOuttake());
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
   * Uses controller app input to set the arm and the elevator to the appropriate setpoints.
   *
   * @return Parallel Command sequence for setting the arm and elevator to the appropriate setpoints
   *     using controller app input
   */
  // public static Command prepScore() {
  //   ControllerOrchestrator controllerApp = RobotContainer.getControllerApp();
  //   return Commands.parallel(
  //       RobotContainer.getElevator().setPositionBlocking(controllerApp.getElevatorSetpoint()),
  //       RobotContainer.getArm().setPositionBlocking(controllerApp.getArmSetpoint()));
  // }

  /**
   * Creates a command to stow the scoring mechanism.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for stowing
   */
  public static Command stow() {
    return Commands.parallel(
            ArmFactory.setPositionBlocking(ArmConstantsLeonidas.ARM_SET_STOW),
            ElevatorFactory.setPositionBlocking(5),
            IntakeFactory.setHomePosition())
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
        // .andThen(LEDFactory.blink())
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
        // .andThen(LEDFactory.auraRizz())
        .withName("Climb");
  }

  public static Command setDefaults() {
    return Commands.parallel(
            ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS),
            ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
            IntakeFactory.setHomePosition())
        .withName("Set defaults");
  }

  public static Command primeL4WhileMoving() {
    return Commands.sequence(
        Commands.waitUntil(
            () -> {
              Pose2d currentPose = RobotContainer.getDrive().getPose();
              for (Pose2d pose : FieldConstants.RED_REEF_POSES.values()) {
                if (NemesisMathUtil.distance(currentPose, pose) < 1.5)
                  return true; // 1.5 meters max distance to start raising elevator
              }
              for (Pose2d pose : FieldConstants.BLUE_REEF_POSES.values()) {
                if (NemesisMathUtil.distance(currentPose, pose) < 1.5)
                  return true; // 1.5 meters max distance to start raising elevator
              }
              return false;
            }),
        primeForLevel(Level.L4) // ,
        );
  }
}
