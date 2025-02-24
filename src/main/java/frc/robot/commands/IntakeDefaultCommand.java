package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.command_factories.IntakeFactory;

public class IntakeDefaultCommand extends Command {
  public IntakeDefaultCommand() {
    addRequirements(RobotContainer.getIntake());
  }

  @Override
  public void initialize() {
    IntakeFactory.defaultCommand().schedule();
  }
}
