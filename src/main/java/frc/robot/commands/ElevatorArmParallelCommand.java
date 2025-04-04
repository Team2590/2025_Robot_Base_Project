package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.IntakeFactory;

public class ElevatorArmParallelCommand extends Command {
  private Command hasCoralCommand =
      Commands.waitSeconds(0.1)
          .andThen(
              IntakeFactory.setPosition(
                  Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS))
          .withName("Intake has coral default command");

  public ElevatorArmParallelCommand() {
    addRequirements(RobotContainer.getArm(), RobotContainer.getElevator());
  }

  @Override
  public void execute() {
    if (RobotState.endEffectorHasGamePiece()) hasCoralCommand.schedule();
    else hasCoralCommand.cancel();
  }

  @Override
  public void end(boolean interrupted) {
    hasCoralCommand.cancel();
  }
}
