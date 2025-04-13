package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState.ReefTargetSide;
import frc.robot.command_factories.ScoringFactory.Level;

public class ControllerFactory {
  public static Command setTargetLevel(Level level) {
    return Commands.runOnce(() -> RobotContainer.getControllerApp().setTarget(level));
  }

  public static Command setTargetSide(ReefTargetSide side) {
    return Commands.runOnce(() -> RobotContainer.getControllerApp().setTarget(side));
  }
}
