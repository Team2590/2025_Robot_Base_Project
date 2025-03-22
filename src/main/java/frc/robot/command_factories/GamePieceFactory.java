package frc.robot.command_factories;

import java.util.Set;

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
    // boolean elevatorSafe = RobotContainer.getElevator().getRotationCount() > Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS;
    // boolean armSafe = RobotContainer.getArm().getAbsolutePosition() > Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION;

    Command[] group = { 
      ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS), 
      ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS) 
    };
    
    // Command moveToHandOffCommand = Commands.defer(() -> elevatorSafe && armSafe ? Commands.parallel(group) : Commands.sequence(group), Set.of(RobotContainer.getElevator(), RobotContainer.getArm()));
    
    Command moveToHandOffCommand = Commands.sequence(group);

    return moveToHandOffCommand
    .alongWith(IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS))
    .andThen(IntakeFactory.runIntake(() -> 6))
    .andThen(IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
    .andThen(
      Commands.parallel(
        IntakeFactory.runIntakeVoltage(() -> 6),
        EndEffectorFactory.runEndEffector()
      )
    );
  }

  public static Command intakeAlgaeL2() {
    return Commands.parallel(Atlas.synchronize(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS, Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS), EndEffectorFactory.runEndEffectorIntakeAlgae());
  }

  public static Command intakeAlgaeL3() {
    return Commands.parallel(Atlas.synchronize(Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS, Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3, Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS), EndEffectorFactory.runEndEffectorIntakeAlgae());

  }
}
