package frc.robot.command_factories;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.util.NemesisMathUtil;

public class GamePieceFactory {
  public static Command intakeCoralFeeder() {
    return new ParallelCommandGroup(
        ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
        ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS),
        EndEffectorFactory.runEndEffector());
  }

  public static Command intakeAlgaeGround() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            IntakeFactory.setIntakeAlgaePosition(),
            IntakeFactory.runIntake(
                () -> Constants.IntakeConstantsLeonidas.INTAKE_ALGAE_INTAKE_SPEED)),
        IntakeFactory.setHoldingAlgaePosition());
  }

  public static Command primeCoralSource() {

    return Commands.parallel(
        ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION),
        ElevatorFactory.setPositionBlocking(
            Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS));
  }

  public static Command intakeCoralGround() {   
    boolean elevatorSafe = RobotContainer.getElevator().getRotationCount() > Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS;
    boolean armSafe = RobotContainer.getArm().getAbsolutePosition() > Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION;
    
    Command moveToHandOffCommand = Commands.either(
      Commands.parallel(
        ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS),
        ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION)
      ),
      Commands.sequence(
        ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS),
        ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION)
      ), 
      () -> elevatorSafe && armSafe
    );

    return moveToHandOffCommand
    .alongWith(IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS))
    .andThen(IntakeFactory.runIntake(() -> 6).until(() -> RobotState.intakeHasCoral()))
    .andThen(IntakeFactory.setPositionBlocking(Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
    .andThen(
      Commands.parallel(
        IntakeFactory.runIntake(() -> -6),
        EndEffectorFactory.runEndEffector()
      ).until(() -> RobotState.endEffectorhasCoral())
    );
  }

  public static Command deAlgaeL2() {
    return new ParallelCommandGroup(
        ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS),
        ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L2),
        EndEffectorFactory.runEndEffectorDeAlgae());
  }

  public static Command deAlgaeL3() {
    return new ParallelCommandGroup(
        ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS),
        ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_DEALGAE_L3),
        EndEffectorFactory.runEndEffectorDeAlgae());
  }
}
