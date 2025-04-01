package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class EndEffectorDefaultCommand extends Command {
  public EndEffectorDefaultCommand() {
    addRequirements(RobotContainer.getEndEffector());
  }

  @Override
  public void execute() {
    RobotContainer.getEndEffector().getIO().setVoltage(-1.5);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.getEndEffector().getIO().stop();
  }
}
