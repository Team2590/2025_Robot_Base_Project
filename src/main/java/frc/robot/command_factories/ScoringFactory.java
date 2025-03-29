package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstantsLeonidas;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringSetpoints;
import frc.robot.commands.MoveFromHandoffCommand;
import frc.robot.util.NemesisMathUtil;

/**
 * Factory class for creating complex scoring-related commands.
 *
 * <p>This class provides methods to create composite commands that coordinate multiple subsystems
 * for scoring operations.
 */
public class ScoringFactory {

  public enum Level {
    L1(
        0 /* Not Using Elevator at L1 */,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW),
    L2(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L2_PRE,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L2_POST),
    L3(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3_PRE,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L3_POST),
    L4(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4,
        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4),
    SOURCE(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS,
        Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION,
        Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
    DEALGAE_L2(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2,
        Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION,
        Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION),
    DEALGAE_L3(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3,
        Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION,
        Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION),
    BARGE(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_BARGE_POS,
        Constants.ArmConstantsLeonidas.ARM_BARGE_POS,
        Constants.ArmConstantsLeonidas.ARM_BARGE_POS),
    PROCESSOR(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_PROCESSOR_POS,
        Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS,
        Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS);

    private final double elevatorSetpoint;
    private final double armPreScoreSetpoint;
    private final double armScoringSetpoint;

    private Level(double elevatorSetpoint, double armPreScoreSetpoint, double armScoringSetpoint) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.armPreScoreSetpoint = armPreScoreSetpoint;
      this.armScoringSetpoint = armScoringSetpoint;
    }

    public double getElevatorSetpoint() {
      return elevatorSetpoint;
    }

    public double getarmPreScoreSetpoint() {
      return armPreScoreSetpoint;
    }

    public double getArmScoringSetpoint() {
      return armScoringSetpoint;
    }
  }

  public static Command score(Level level) {
    return switch (level) {
      case L1:
        yield scoreL1();
      case L2:
        yield primeForLevel(level)
            .andThen(
                new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L2_POST))
            .withName("Score " + level.name());
      case L3:
        yield primeForLevel(level)
            .andThen(
                Commands.parallel(
                    IntakeFactory.setPositionBlocking(
                        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                    ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                    ArmFactory.setPositionBlocking(
                        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L3_POST)))
            .withName("Score " + level.name());
      case L4:
        yield primeForLevel(level)
            .andThen(
                Commands.parallel(
                    IntakeFactory.setPositionBlocking(
                        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                    ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                    ArmFactory.setPositionBlocking(
                        Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POSE_L4_POST)))
            .withName("Score " + level.name());
      default:
        yield primeForLevel(level)
            .andThen(EndEffectorFactory.runEndEffectorOuttake())
            .until(() -> !RobotState.endEffectorHasGamePiece())
            .withName("Score " + level.name());
    };
  }

  public static Command primeForLevel(Level level) {
    switch (level) {
      case L4:
        return Commands.parallel(
                Commands.print("Priming " + level.name()),
                IntakeFactory.setPositionBlocking(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                ArmFactory.setPositionBlocking(
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4))
            .withName("Prime " + level.name());
      case L3:
        return Commands.parallel(
                Commands.print("Priming " + level.name()),
                new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3_PRE))
            .withName("Prime " + level.name());
      case L2:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L2_PRE)
                .withName("Prime " + level.name()));
      default:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3_PRE)
                .withName("Prime " + level.name()));
    }
  }

  public static Command score(ScoringSetpoints setpoints) {
    return primeForLevel(setpoints)
        .andThen(EndEffectorFactory.runEndEffectorOuttake())
        .until(() -> !RobotState.endEffectorHasGamePiece())
        .withName(
            "Score with Elevator setpoint "
                + setpoints.elevatorSetpoint
                + " and arm setpoint = "
                + setpoints.armSetpoint);
  }

  public static Command primeForLevel(ScoringSetpoints setpoints) {
    return Commands.parallel(
            Commands.print(
                "Priming with Elevator setpoint "
                    + setpoints.elevatorSetpoint
                    + " and arm setpoint = "
                    + setpoints.armSetpoint),
            new MoveFromHandoffCommand(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                setpoints.elevatorSetpoint,
                setpoints.armSetpoint))
        .withName(
            "Priming with Elevator setpoint "
                + setpoints.elevatorSetpoint
                + " and arm setpoint = "
                + setpoints.armSetpoint);
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

  public static Command scoreAlgaeBarge() {
    return ElevatorFactory.setPositionRun(Constants.ElevatorConstantsLeonidas.ELEVATOR_BARGE_POS)
        .alongWith(ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_BARGE_POS));
  }

  /**
   * Creates a command sequence for scoring at processor.
   *
   * @return Command sequence for processor scoring
   */
  public static Command scoreProcessor() {
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Level.PROCESSOR.getElevatorSetpoint(),
            Level.PROCESSOR.getArmScoringSetpoint())
        .withName("Score Processor");
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
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
            Constants.ArmConstantsLeonidas.ARM_SET_STOW)
        .withName("Stow");
  }

  public static Command prepClimb() {
    // return new ParallelCommandGroup(
    // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS),
    // ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
    return new MoveFromHandoffCommand(
            IntakeConstantsLeonidas.INTAKE_FACTORY_CORAL_POSITION,
            .33,
            Constants.ArmConstantsLeonidas.ARM_SET_STOW)
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
            ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS),
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
