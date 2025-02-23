package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ArmFactory;

public class ArmDefaultCommand extends Command {
  public ArmDefaultCommand() {
    addRequirements(RobotContainer.getArm());
  }

  @Override
  public void initialize() {
    ArmFactory.defaultCommand().schedule();
  }
}
