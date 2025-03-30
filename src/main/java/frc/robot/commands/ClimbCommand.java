package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import java.util.Set;

public class ClimbCommand extends Command {

  private Trigger limitSwitch;

  public ClimbCommand() {
    setName("Climb Command");
    addRequirements(
        Set.of(
            RobotContainer.getArm(),
            RobotContainer.getElevator(),
            RobotContainer.getClimb(),
            RobotContainer.getIntake()));
  }

  @Override
  public void execute() {}
}
