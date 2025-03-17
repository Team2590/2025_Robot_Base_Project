package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ArmFactory;

public class ArmDefaultCommand extends Command {
  private Command hasCoralCommand =
      ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS - 0.2)
          .withName("Arm has coral default command");
  private Command notHasCoralCommand =
      ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POSITION)
          .withName("Arm not has coral default command");

  public ArmDefaultCommand() {
    addRequirements(RobotContainer.getArm());
  }

  @Override
  public void execute() {
    if (RobotState.endEffectorhasCoral()) {
      hasCoralCommand.schedule();
    } else {
      notHasCoralCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    hasCoralCommand.cancel();
    notHasCoralCommand.cancel();
  }
}
