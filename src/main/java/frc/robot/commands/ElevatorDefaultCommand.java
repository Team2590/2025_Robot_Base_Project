package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  private Trigger condition = new Trigger(() -> RobotState.isEndEffectorhasCoral());
  private Command command = ElevatorFactory.defaultCommand();

  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  @Override
  public void initialize() {
    condition.onTrue(command);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) command.cancel();
  } 
}
