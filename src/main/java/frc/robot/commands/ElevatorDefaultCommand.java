package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  @Override
  public void initialize() {
    ElevatorFactory.defaultCommand().schedule();
  }
}
