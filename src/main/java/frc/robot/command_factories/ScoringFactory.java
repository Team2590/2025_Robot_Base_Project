package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstantsLeonidas;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringSetpoints;
import frc.robot.commands.MoveFromHandoffCommand;
import java.util.Set;

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
        Constants.ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_PRE,
        Constants.ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_POST),
    L3(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS,
        Constants.ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_PRE,
        Constants.ArmConstantsLeonidas.ARM_SCORE_FRONT_BACK_POST),
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
        Constants.ArmConstantsLeonidas.ARM_BARGE_FRONT_FRONT_POS,
        Constants.ArmConstantsLeonidas.ARM_BARGE_FRONT_FRONT_POS),
    PROCESSOR(
        Constants.ElevatorConstantsLeonidas.ELEVATOR_PROCESSOR_POS,
        Constants.ArmConstantsLeonidas.ARM_BARGE_BACK_FRONT_POS,
        Constants.ArmConstantsLeonidas.ARM_BARGE_BACK_FRONT_POS);

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
    return Commands.defer(
        () -> {
          return switch (level) {
            case L1:
              yield scoreL1();
            case L2:
              yield primeForLevel(level)
                  .andThen(
                      new MoveFromHandoffCommand(
                          Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                          level.getElevatorSetpoint(),
                          RobotState.getInstance().getCoralScoringSetpoints().armPlaceSetpoint))
                  .alongWith(RobotContainer.getEndEffector().stopEndEffector())
                  .withName("Score " + level.name());
            case L3:
              yield primeForLevel(level)
                  .andThen(
                      Commands.parallel(
                          IntakeFactory.setPositionBlocking(
                              Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                          ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                          ArmFactory.setPositionBlocking(
                              RobotState.getInstance()
                                  .getCoralScoringSetpoints()
                                  .armPlaceSetpoint)))
                  .alongWith(RobotContainer.getEndEffector().stopEndEffector())
                  .withName("Score " + level.name());
            case L4:
              yield primeForLevel(level)
                  .andThen(
                      Commands.parallel(
                          IntakeFactory.setPositionBlocking(
                              Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                          ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                          ArmFactory.setPositionBlocking(
                              RobotState.getInstance()
                                  .getCoralScoringSetpoints()
                                  .armPlaceSetpoint)))
                  .alongWith(RobotContainer.getEndEffector().stopEndEffector())
                  .withName("Score " + level.name());
            default:
              yield primeForLevel(level)
                  .andThen(EndEffectorFactory.runEndEffectorOuttake())
                  .until(() -> !RobotState.endEffectorHasGamePiece())
                  .alongWith(RobotContainer.getEndEffector().stopEndEffector())
                  .withName("Score " + level.name());
          };
        },
        Set.of(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake()));
  }

  public static Command primeForLevel(Level level) {
    return Commands.defer(
        () -> {
          switch (level) {
            case L4:
              return Commands.parallel(
                      // Commands.print("Priming " + level.name()),
                      IntakeFactory.setPositionBlocking(
                          Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS),
                      ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
                      ArmFactory.setPositionBlocking(
                          RobotState.getInstance().getCoralScoringSetpoints().armSetpoint))
                  .withName("Prime " + level.name());
            case L3:
              return IntakeFactory.setPositionBlocking(
                      Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS)
                  .andThen(
                      Commands.parallel(
                          // Commands.print("Priming " + level.name()),
                          new MoveFromHandoffCommand(
                              Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                              level.getElevatorSetpoint(),
                              RobotState.getInstance().getCoralScoringSetpoints().armSetpoint)))
                  .withName("Prime " + level.name());
            case L2:
              return IntakeFactory.setPositionBlocking(
                      Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS)
                  .andThen(
                      Commands.parallel(
                          // Commands.print("Priming " + level.name()),
                          new MoveFromHandoffCommand(
                              Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                              level.getElevatorSetpoint(),
                              RobotState.getInstance().getCoralScoringSetpoints().armSetpoint)))
                  .withName("Prime " + level.name());
            default:
              return Commands.parallel(
                  // Commands.print("Priming " + level.name()),
                  new MoveFromHandoffCommand(
                          Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                          level.getElevatorSetpoint(),
                          RobotState.getInstance().getCoralScoringSetpoints().armSetpoint)
                      .withName("Prime " + level.name()));
          }
        },
        Set.of(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake()));
  }

  public static Command score(ScoringSetpoints setpoints) {
    return primeForLevel(setpoints)
        .andThen(ArmFactory.setPositionBlocking(setpoints.armPlaceSetpoint))
        .withName(
            "Score with Elevator setpoint "
                + setpoints.elevatorSetpoint
                + " and arm setpoint = "
                + setpoints.armSetpoint);
  }

  public static Command primeForLevel(ScoringSetpoints setpoints) {
    return Commands.parallel(
            // Commands.print(
            //     "Priming with Elevator setpoint "
            //         + setpoints.elevatorSetpoint
            //         + " and arm setpoint = "
            //         + setpoints.armSetpoint),
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
        IntakeFactory.runIntake(() -> 5).withName("Score L1"));
  }

  public static Command scoreAlgaeBarge() {

    return Commands.defer(
        () -> {
          return Commands.parallel(
                  Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(true)),
                  EndEffectorFactory.runEndEffectorVoltage(
                      Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE),
                  ElevatorFactory.setPositionRun(
                      Constants.ElevatorConstantsLeonidas.ELEVATOR_BARGE_POS),
                  ArmFactory.setPositionRun(RobotState.getInstance().getBargeArmPos()))
              .withName("Score Algae Barge");
        },
        Set.of(
            RobotContainer.getElevator(),
            RobotContainer.getArm(),
            RobotContainer.getEndEffector()));
  }

  /**
   * Creates a command sequence for scoring at processor.
   *
   * @return Command sequence for processor scoring
   */
  public static Command scoreProcessor() {
    return Commands.parallel(
            EndEffectorFactory.runEndEffectorVoltage(
                Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE),
            new MoveFromHandoffCommand(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                Level.PROCESSOR.getElevatorSetpoint(),
                RobotState.getInstance().getProcessorArmPos()))
        .withName("Score Processor");
  }

  /**
   * Creates a command to stow the scoring mechanism.
   *
   * @param container The RobotContainer instance
   * @return Command sequence for stowing
   */
  public static Command stow() {
    return IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS)
        .andThen(
            Commands.defer(
                () -> {
                  return new MoveFromHandoffCommand(
                          Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                          Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
                          RobotState.getInstance().getStowSetpoint())
                      .withName("Stow");
                },
                Set.of(
                    RobotContainer.getArm(),
                    RobotContainer.getElevator(),
                    RobotContainer.getIntake())));
  }

  public static Command prepClimb() {
    // return new ParallelCommandGroup(
    // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS),
    // ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
    return Commands.defer(
            () -> {
              return new MoveFromHandoffCommand(
                  IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
                  .33,
                  RobotState.getInstance().getStowSetpoint());
            },
            Set.of(
                RobotContainer.getArm(), RobotContainer.getIntake(), RobotContainer.getElevator()))
        // .andThen(LEDFactory.blink())
        .withName("Deploy climb mechanism");

    // , ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION)
  }

  public static Command deployMech() {
    return ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION);
  }

  public static Command climb() {
    return ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION);
  }

  public static Command setDefaults() {
    return Commands.parallel(
            ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS),
            ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS),
            IntakeFactory.setHomePosition())
        .withName("Set defaults");
  }

  public static Command scoreL4Sequentially() {
    return Commands.defer(
        () -> {
          return new SequentialCommandGroup(
              // ArmFactory.setPositionBlocking(RobotState.getInstance().getStowSetpoint()),
              ElevatorFactory.setPositionBlocking(
                  Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS),
              ArmFactory.setPositionBlocking(
                  RobotState.getInstance().getCoralScoringSetpoints().armSetpoint),
              Commands.waitSeconds(.25),
              ArmFactory.setPositionBlocking(
                  RobotState.getInstance().getCoralScoringSetpoints().armPlaceSetpoint));
        },
        Set.of(RobotContainer.getArm(), RobotContainer.getElevator()));
  }

  public static Command armFollowThrough() {
    return Commands.defer(
        () ->
            ArmFactory.setPosition(
                RobotState.getInstance().getCoralScoringSetpoints().armPlaceSetpoint),
        Set.of(RobotContainer.getArm()));
  }
}
