package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.EndEffectorFactory;
import frc.robot.util.NemesisMathUtil;

public class EndEffectorDefaultCommand extends Command {
  private Command notHasCoralCommand =
      Commands.waitSeconds(0.1)
          .andThen(EndEffectorFactory.runEndEffector())
          .withName("EndEffector has coral default command");
  private double DISTANCE_THRESHOLD = 2.0;

  public EndEffectorDefaultCommand() {
    addRequirements(RobotContainer.getEndEffector());
  }

  @Override
  public void execute() {
    Commands.print(RobotContainer.getDrive().getPose().toString());
    if (!RobotState.endEffectorhasCoral()
        && NemesisMathUtil.isNearSource(
            () -> RobotContainer.getDrive().getPose(), DISTANCE_THRESHOLD))
      notHasCoralCommand.schedule();
    else notHasCoralCommand.cancel();
  }

  @Override
  public void end(boolean interrupted) {
    notHasCoralCommand.cancel();
  }
}
