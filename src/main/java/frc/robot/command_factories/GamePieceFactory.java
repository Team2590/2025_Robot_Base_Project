package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.util.Atlas;
import frc.robot.util.NemesisTimedCommand;

public class GamePieceFactory {

  public static Command intakeAlgaeGround() {
    return Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
            Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS)
        .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae());
  }

  public static Command intakeCoralGroundandHandoff() {
    return Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS,
            Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS)
        .andThen(
            NemesisTimedCommand.generateTimedCommand(
                IntakeFactory.runIntakeVoltage(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED),
                3))
        .andThen(
            Atlas.synchronize(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS,
                Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS,
                Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS))
        .andThen(
            Commands.race(
                EndEffectorFactory.runEndEffector(),
                IntakeFactory.runIntakeVoltage(
                    () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED)))
        .andThen(
            Atlas.synchronize(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS,
                Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_TRANSITION_POS,
                Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS));
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

  public static Command intakeCoralGround() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            IntakeFactory.setIntakeCoralPosition(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED)),
        IntakeFactory.setHoldingAlgaePosition());
  }

  public static Command GrabAlgaeL2(EndEffector endEffector, Arm arm, Elevator elevator) {
    return new GrabAlgaeCommand(Level.DEALGAE_L2, endEffector, arm, elevator);
  }

  public static Command GrabAlgaeL3(EndEffector endEffector, Arm arm, Elevator elevator) {
    return new GrabAlgaeCommand(Level.DEALGAE_L3, endEffector, arm, elevator);
  }
}
