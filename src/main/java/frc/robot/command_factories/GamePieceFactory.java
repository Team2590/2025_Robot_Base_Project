package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.util.Atlas;

public class GamePieceFactory {
  // public static Command intakeCoralFeeder() {
  //   return new ParallelCommandGroup(
  //       ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
  //       ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS),
  //       EndEffectorFactory.runEndEffector());
  // }

  public static Command intakeAlgaeGround() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            IntakeFactory.setIntakeAlgaePosition(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_ALGAE_INTAKE_SPEED)),
        IntakeFactory.setHoldingAlgaePosition());
  }

  // public static Command primeCoralSource() {

  //   return Commands.parallel(
  //       ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
  //       ElevatorFactory.setPositionBlocking(
  //           Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS));
  // }

  public static Command primeCoralGround() {
    return Atlas.synchronize(Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS, Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS, Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS);
  }

  public static Command intakeCoralGround() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            IntakeFactory.setIntakeCoralPosition(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_INTAKE_SPEED)),
        IntakeFactory.setHoldingAlgaePosition());
  }

  public static Command intakeAlgaeL2() {
    return Commands.parallel(Atlas.synchronize(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS, Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS), EndEffectorFactory.runEndEffectorIntakeAlgae());
  }

  public static Command intakeAlgaeL3() {
    return Commands.parallel(Atlas.synchronize(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS, Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS), EndEffectorFactory.runEndEffectorIntakeAlgae());

  }
}
