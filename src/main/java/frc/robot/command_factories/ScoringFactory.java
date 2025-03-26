package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringSetpoints;
import frc.robot.util.Atlas;
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
    L1(0 /* Not Using Elevator at L1 */, Constants.ArmConstantsLeonidas.ARM_SET_STOW, Constants.ArmConstantsLeonidas.ARM_SET_STOW),
    L2(Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3, Constants.ArmConstantsLeonidas.ARM_CORAL_RELEASE_SETPOINT),
    L3(Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3, Constants.ArmConstantsLeonidas.ARM_CORAL_RELEASE_SETPOINT),
    L4(Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4, Constants.ArmConstantsLeonidas.ARM_CORAL_RELEASE_SETPOINT),
    SOURCE(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS, Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION, Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
    DEALGAE_L2(Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2, Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION, Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION),
    DEALGAE_L3(Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3, Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION, Constants.ArmConstantsLeonidas.ARM_DEALGAE_POSITION),
    BARGE(Constants.ElevatorConstantsLeonidas.ELEVATOR_BARGE_POS, Constants.ArmConstantsLeonidas.ARM_BARGE_POS, Constants.ArmConstantsLeonidas.ARM_BARGE_POS),
    PROCESSOR(Constants.ElevatorConstantsLeonidas.ELEVATOR_PROCESSOR_POS, Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS,Constants.ArmConstantsLeonidas.ARM_PROCESSOR_POS);

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

    public double getarmPreScoreSetpoint(){
      return armPreScoreSetpoint;
    }

    public double getArmScoringSetpoint(){
      return armScoringSetpoint;
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
                Atlas.synchronize(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4))
            .withName("Prime " + level.name());
      case L3:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            Atlas.synchronize(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3)
                .withName("Prime " + level.name()));
      case L2:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            Atlas.synchronize(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    level.getElevatorSetpoint(),
                    Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3)
                .withName("Prime " + level.name()));
      default:
        return Commands.parallel(
            Commands.print("Priming " + level.name()),
            ElevatorFactory.setPositionBlocking(level.getElevatorSetpoint()),
            ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3)
                .withName("Prime " + level.name()));
    }
  }

  public static Command score(ScoringSetpoints setpoints) {
    return primeForLevel(setpoints)
        .andThen(EndEffectorFactory.runEndEffectorOuttake())
        .until(() -> !RobotState.endEffectorhasCoral())
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
        ElevatorFactory.setPositionBlocking(setpoints.elevatorSetpoint),
        ArmFactory.setPositionBlocking(setpoints.armSetpoint)
            .withName(
                "Priming with Elevator setpoint "
                    + setpoints.elevatorSetpoint
                    + " and arm setpoint = "
                    + setpoints.armSetpoint));
  }

  // public static Command scoreTeleop(Level level) {
  //   return switch (level) {
  //     case L1:
  //       yield scoreL1();
  //     default:
  //       yield primeForLevelTeleop(level).withName("Score " + level.name());
  //   };
  // }

  // public static Command primeForLevelTeleop(Level level) {
  //   switch (level) {
  //     case L4:
  //       return Commands.sequence(
  //               Commands.parallel(
  //                   Commands.print("Priming " + level.name()),
  //                   ElevatorFactory.setPositionRun(level.getElevatorPosition())),
  //
  // ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4))
  //           .withName("Prime " + level.name());
  //     case L3:
  //       return Commands.sequence(
  //               Commands.parallel(
  //                   Commands.print("Priming " + level.name()),
  //                   ElevatorFactory.setPositionRun(level.getElevatorPosition())),
  //
  // ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3))
  //           .withName("Prime " + level.name());
  //     default:
  //       return Commands.sequence(
  //               Commands.parallel(
  //                   Commands.print("Priming " + level.name()),
  //                   ElevatorFactory.setPositionRun(level.getElevatorPosition())),
  //               ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS))
  //           .withName("Prime " + level.name());
  //   }
  // }

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
        .alongWith(ArmFactory.setPositionRun(Constants.ArmConstantsLeonidas.ARM_BARGE_POS));
  }

  /**
   * Creates a command sequence for scoring at processor.
   *
   * @return Command sequence for processor scoring
   */
  public static Command scoreProcessor() {
    return Atlas.synchronize(
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
    return Atlas.synchronize(
        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW).withName("Stow");
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
