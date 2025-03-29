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
              ArmFactory.setPosition(
                  Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L3_PRE - 0.2))
          .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
          .withName("Arm has coral default command");
  private Command notHasCoralCommand =
      ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS)
          .withName("Arm not has coral default command");

  public ArmDefaultCommand() {
    addRequirements(RobotContainer.getArm());
  }

  @Override
  public void execute() {
    if (RobotState.endEffectorHasGamePiece()) {
      hasCoralCommand.schedule();
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
