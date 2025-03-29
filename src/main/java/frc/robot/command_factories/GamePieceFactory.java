package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.MoveFromHandoffCommand;
import frc.robot.commands.MoveToHandoffCommand;

public class GamePieceFactory {

  public static Command intakeAlgaeGround() {
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
            Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS)
        .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae());
  }

  public static Command intakeCoralGroundAndHandoff() {
    return new MoveToHandoffCommand()
        .andThen(
            Commands.parallel(
                IntakeFactory.runIntake(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED),
                Commands.waitUntil(() -> RobotContainer.getIntake().detectCoral())
                    .andThen(
                        Commands.parallel(
                            ElevatorFactory.setPositionBlocking(
                                Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS),
                            IntakeFactory.setPositionBlocking(
                                Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS)))))
        .andThen(
            Commands.race(
                EndEffectorFactory.runEndEffector(),
                IntakeFactory.runIntakeVoltage(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED)))
        .onlyIf(() -> !RobotState.endEffectorHasGamePiece());
  }

  public static Command intakeCoralNoHandoff() {
    return Commands.parallel(
            IntakeFactory.setPositionBlocking(
                Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED))
        .until(() -> RobotContainer.getIntake().hasCoral())
        .andThen(
            IntakeFactory.setPositionBlocking(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS));
  }

  public static Command GrabAlgaeL2() {
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).elevatorSetpoint,
            RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).armPlaceSetpoint)
        .alongWith(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae())
        .until(() -> RobotState.endEffectorHasGamePiece())
        .andThen(
            Commands.parallel(
                ElevatorFactory.setPositionBlocking(
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS),
                ArmFactory.setPositionBlocking(
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS)));
  }

  public static Command GrabAlgaeL3() {
    return new MoveFromHandoffCommand(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L3).elevatorSetpoint,
            RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L3).armPlaceSetpoint)
        .alongWith(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae())
        .until(() -> RobotState.endEffectorHasGamePiece())
        .andThen(
            Commands.parallel(
                ElevatorFactory.setPositionBlocking(
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS),
                ArmFactory.setPositionBlocking(
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS)));
  }
}
