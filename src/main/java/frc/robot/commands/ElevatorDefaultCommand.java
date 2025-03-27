package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  private Command notHasCoralCommand =
      ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS)
          .withName("Elevator not has coral default command");

  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  @Override
  public void execute() {
    if (!RobotState.endEffectorHasGamePiece()) notHasCoralCommand.schedule();
    else notHasCoralCommand.cancel();
  }

  @Override
  public void end(boolean interrupted) {
    notHasCoralCommand.cancel();
  }
}
