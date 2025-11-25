package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.MoveFromHandoffCommand;
import java.util.Set;

public class GamePieceFactory {

  public static Command intakeAlgaeGround() {
    // spotless:off
    return Commands.defer(() -> {
        return new MoveFromHandoffCommand(
                    Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                    Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
                    RobotState.getInstance().getGroundPickupArmPos() // 0.58
                )
                .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae().alongWith(Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(true))))
                // .andThen(
                //     Commands.parallel(
                //         ScoringFactory.stow(),
                //         EndEffectorFactory.runEndEffectorVoltage(Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE)
                //     )
                // )
        .withName("Intake Algae Ground");
    }, Set.of(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getEndEffector()));
    // spotless:on
  }

  public static Command intakeAlgaeGroundNoStow() {
    return Commands.defer(
        () -> {
          return new MoveFromHandoffCommand(
                  Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                  Constants.ElevatorConstantsLeonidas.ELEVATOR_INTAKE_ALGAE_POS,
                  RobotState.getInstance().getGroundPickupArmPos())
              .andThen(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae())
              .andThen(Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(true)))
              .withName("Intake Algae Ground No Stow");
        },
        Set.of(
            RobotContainer.getElevator(),
            RobotContainer.getArm(),
            RobotContainer.getEndEffector()));
  }

  public static Command finishHandoff() {
    return Commands.defer(
        () -> {
          Command moveArmAndElevatorCommand;

          if (RobotContainer.getElevator().getRotationCount()
              > Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
            moveArmAndElevatorCommand =
                Commands.parallel(
                    ArmFactory.setPositionBlocking(ArmConstantsLeonidas.ARM_HANDOFF_POS),
                    ElevatorFactory.setPosition(
                        Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS));
          } else {
            moveArmAndElevatorCommand =
                Commands.sequence(
                    ElevatorFactory.setPosition(
                        Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS),
                    ArmFactory.setPositionBlocking(ArmConstantsLeonidas.ARM_HANDOFF_POS));
          }

          return moveArmAndElevatorCommand
              .andThen(
                  IntakeFactory.setPositionBlocking(
                      Constants.IntakeArmConstantsLeonidas.INTAKE_HANDOFF_POS))
              .andThen(
                  ElevatorFactory.setPositionBlocking(
                      Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_PRE_POS))
              .andThen(
                  Commands.parallel(
                      EndEffectorFactory.runEndEffectorVoltage(
                          -Constants.EndEffectorConstantsLeonidas.INTAKE_VOLTAGE),
                      IntakeFactory.runIntakeVoltage(
                          () -> Constants.IntakeConstantsLeonidas.INTAKE_CORAL_OUTTAKE_SPEED)),
                  Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(false))
                      .until(() -> RobotState.endEffectorHasGamePiece()))
              .andThen(
                  ElevatorFactory.setPositionBlocking(
                      Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS))
              .withName("Finish Handoff");
        },
        Set.of(RobotContainer.getEndEffector(), RobotContainer.getIntake()));
  }

  public static Command grabAlgaeReef() {
    // spotless:off
    return Commands.defer(
        () -> {
          return 
                new MoveFromHandoffCommand(
                        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
                        RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).elevatorSetpoint,
                        RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).armSetpoint
                )
                .alongWith(EndEffectorFactory.runEndEffectorGrabAndHoldAlgae())
                .alongWith(Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(true)))
                .withName("Grab Algae Reef");
        },
        Set.of(RobotContainer.getArm(), RobotContainer.getElevator(), RobotContainer.getEndEffector())
    );
    // spotless:on
  }
}
