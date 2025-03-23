package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
            Constants.ArmConstantsLeonidas.ARM_INTAKE_ALGAE_GROUND_POSITION)
        .andThen(EndEffectorFactory.runEndEffectorIntakeAlgae());
  }

  // public static Command primeCoralSource() {

  //   return Commands.parallel(
  //
  // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
  //       ElevatorFactory.setPositionBlocking(
  //           Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS));
  // }

  public static Command primeCoralGround() {
    // probably is obsolete now, just use intakeCoralGround() for autos
    return Atlas.synchronize(
        Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS,
        Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS);
  }

  public static Command intakeCoralGround() {
    // boolean elevatorSafe = RobotContainer.getElevator().getRotationCount() >
    // Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS;
    // boolean armSafe = RobotContainer.getArm().getAbsolutePosition() >
    // Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION;
    return Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS,
            Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS)
        .andThen(IntakeFactory.runIntake(() -> 6))
        .andThen(
            IntakeFactory.setPositionBlocking(
                Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
        .andThen(
            Commands.parallel(
                IntakeFactory.runIntakeVoltage(() -> 6), EndEffectorFactory.runEndEffector()));
  }

  public static Command intakeAlgaeL2() {
    return Commands.parallel(
        Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS),
        EndEffectorFactory.runEndEffectorIntakeAlgae());
  }

  public static Command intakeAlgaeL3() {
    return Commands.parallel(
        Atlas.synchronize(
            Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS),
        EndEffectorFactory.runEndEffectorIntakeAlgae());
  }
}
