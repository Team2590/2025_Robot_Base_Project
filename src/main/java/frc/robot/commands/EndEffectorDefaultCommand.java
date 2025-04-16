package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class EndEffectorDefaultCommand extends Command {
  public EndEffectorDefaultCommand() {
    addRequirements(RobotContainer.getEndEffector());
  }

  @Override
  public void execute() {
    if (RobotState.getInstance().hasAlgae()) {
      RobotContainer.getEndEffector()
          .getIO()
          .setVoltage(Constants.EndEffectorConstantsLeonidas.HOLD_ALGAE_VOLTAGE);
    } else {
      RobotContainer.getEndEffector().getIO().setVoltage(Constants.EndEffectorConstantsLeonidas.HOLD_CORAL_VOLTAGE); // -.3
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.getEndEffector().getIO().stop();
  }
}
