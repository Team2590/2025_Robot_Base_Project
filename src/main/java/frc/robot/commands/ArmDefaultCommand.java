package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.ArmFactory;

public class ArmDefaultCommand extends Command {
  private Command hasCoralCommand =
      Commands.waitSeconds(0.2)
          .andThen(
              ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS - 0.2))
          .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
          .withName("Arm has coral default command");
  private Command notHasCoralCommand =
      Commands.waitSeconds(0.2)
          .andThen(
              ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION))
          .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
          .withName("Arm not has coral default command");

  public ArmDefaultCommand() {
    addRequirements(RobotContainer.getArm());
  }

  @Override
  public void execute() {
    if (RobotState.endEffectorhasCoral()) {
      hasCoralCommand.execute();
    } else {
      notHasCoralCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    hasCoralCommand.cancel();
    notHasCoralCommand.cancel();
  }
}
