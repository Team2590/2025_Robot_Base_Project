package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  private Command command =
      Commands.waitSeconds(0.2)
          .andThen(
              ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS))
          .withName("Elevator not has coral default command")
          .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  @Override
  public void execute() {
    command.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }
}
