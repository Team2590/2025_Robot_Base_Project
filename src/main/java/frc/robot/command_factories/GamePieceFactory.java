package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.MoveFromHandoffCommand;
import frc.robot.commands.MoveToHandoffCommand;
import java.util.Set;

public class GamePieceFactory {

  public static Command intakeAlgaeGround() {
    return Commands.parallel(
            new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
                    Constants.ArmConstantsLeonidas.ARM_INTAKE_ALGAE_POS)
                .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae())
                .andThen(
                    Commands.parallel(
                        ScoringFactory.stow(),
                        EndEffectorFactory.runEndEffectorVoltage(
                            Constants.EndEffectorConstantsLeonidas
                                .HOLD_ALGAE_VOLTAGE)))) // .until(()->
        // RobotContainer.getEndEffector().hasGamePiece()))
        // .andThen(ScoringFactory.stow())
        .withName("Intake Algae Ground");
  }

  public static Command intakeAlgaeGroundNoStow() {
    return Commands.parallel(
            new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
                    Constants.ArmConstantsLeonidas.ARM_INTAKE_ALGAE_POS)
                .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae()))
        .withName("Intake Algae Ground No Stow");
  }

  public static Command intakeUprightCoralNoStow() {
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
            Constants.ArmConstantsLeonidas.ARM_INTAKE_ALGAE_POS)
        .andThen(
            EndEffectorFactory.runEndEffectorVoltage(-12)
                .until(() -> RobotState.endEffectorHasGamePiece()))
        .andThen(ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SET_STOW));
  }

  public static Command intakeCoralGroundAndHandoff() {
    return Commands.sequence(
            new MoveToHandoffCommand(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED))
        .andThen(
            Commands.parallel(
                IntakeFactory.runIntake(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED),
                Commands.waitUntil(() -> RobotContainer.getIntake().detectCoral())
                    .andThen(
                        IntakeFactory.setPositionBlocking(
                            Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
                    .andThen(ElevatorFactory.setPositionBlocking(13.9))))
        .andThen(
            Commands.parallel(
                    EndEffectorFactory.runEndEffectorVoltage(
                        -Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE),
                    IntakeFactory.runIntakeVoltage(
                        () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED))
                .until(() -> RobotState.endEffectorHasGamePiece()))
        .andThen(
            ElevatorFactory.setPositionBlocking(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS))
        .withName("Handoff");
  }

  public static Command intakeCoralGroundAndHandoffNoStow() {
    return Commands.sequence(
            new MoveToHandoffCommand(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED))
        .andThen(
            Commands.parallel(
                IntakeFactory.runIntake(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED),
                Commands.waitUntil(() -> RobotContainer.getIntake().detectCoral())
                    .andThen(
                        IntakeFactory.setPositionBlocking(
                            Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
                    .andThen(ElevatorFactory.setPositionBlocking(13.9))))
        .andThen(
            Commands.parallel(
                    EndEffectorFactory.runEndEffectorVoltage(
                        -Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE),
                    IntakeFactory.runIntakeVoltage(
                        () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED))
                .until(() -> RobotState.endEffectorHasGamePiece()))
        .andThen(
            ElevatorFactory.setPositionBlocking(
                Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS))
        .withName("Handoff");
  }

  public static Command intakeCoralNoHandoff() {
    return Commands.parallel(
            IntakeFactory.setPositionBlocking(
                Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED))
        .until(() -> RobotContainer.getIntake().hasCoral())
        .andThen(
            IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS))
        .withName("Intake Coral No Handoff");
  }

  public static Command GrabAlgaeL2() {

    return Commands.defer(
        () -> {
          return (new MoveFromHandoffCommand(
                      Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                      RobotState.getInstance()
                          .getDealgaeSetpoints(Level.DEALGAE_L2)
                          .elevatorSetpoint,
                      RobotState.getInstance()
                          .getDealgaeSetpoints(Level.DEALGAE_L2)
                          .armPlaceSetpoint)
                  .alongWith(
                      EndEffectorFactory.runEndEffectorVoltage(
                              Constants.EndEffectorConstantsLeonidas.INTAKE_ALGAE_VOLTAGE)
                          .until(() -> RobotState.endEffectorHasGamePiece())))
              .andThen(
                  EndEffectorFactory.runEndEffectorVoltage(
                      Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE))
              .withName("Grab Algae L2");
        },
        Set.of(
            RobotContainer.getArm(),
            RobotContainer.getElevator(),
            RobotContainer.getEndEffector()));
  }

  public static Command GrabAlgaeL3() {
    return Commands.defer(
        () -> {
          return (new MoveFromHandoffCommand(
                      Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                      RobotState.getInstance()
                          .getDealgaeSetpoints(Level.DEALGAE_L3)
                          .elevatorSetpoint,
                      RobotState.getInstance()
                          .getDealgaeSetpoints(Level.DEALGAE_L3)
                          .armPlaceSetpoint)
                  .alongWith(
                      EndEffectorFactory.runEndEffectorVoltage(
                              Constants.EndEffectorConstantsLeonidas.INTAKE_ALGAE_VOLTAGE)
                          .until(() -> RobotState.endEffectorHasGamePiece())))
              .andThen(
                  EndEffectorFactory.runEndEffectorVoltage(
                      Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE))
              .withName("Grab Algae L2");
        },
        Set.of(
            RobotContainer.getArm(),
            RobotContainer.getElevator(),
            RobotContainer.getEndEffector()));
  }
}
