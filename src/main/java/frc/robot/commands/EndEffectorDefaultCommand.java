package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.EndEffectorFactory;

public class EndEffectorDefaultCommand extends Command {
  private Command notHasCoralCommand =
      Commands.waitSeconds(0.2)
          .andThen(EndEffectorFactory.runEndEffector())
          .withName("EndEffector not has coral default command");

  public EndEffectorDefaultCommand() {
    addRequirements(RobotContainer.getEndEffector());
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
