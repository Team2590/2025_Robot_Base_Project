package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class EndEffectorDefaultCommand extends Command {
  public EndEffectorDefaultCommand() {
    addRequirements(RobotContainer.getEndEffector());
  }

  @Override
  public void execute() {
    double voltage = RobotState.getInstance().getEndEffectorCurrent();
    RobotContainer.getEndEffector().getIO().setVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.getEndEffector().getIO().stop();
  }
}
