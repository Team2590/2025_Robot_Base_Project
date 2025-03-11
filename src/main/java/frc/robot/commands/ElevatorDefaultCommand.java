package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  private Command hasCoralCommand =
      ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS);
  private Command notHasCoralCommand =
      ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS);

  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  @Override
  public void execute() {
    if (RobotState.endEffectorhasCoral()) {
      hasCoralCommand.schedule();
    } else {
      notHasCoralCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    hasCoralCommand.cancel();
    notHasCoralCommand.cancel();
  }
}
