package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ArmFactory;

public class ArmDefaultCommand extends Command {
  private static Trigger condition = new Trigger(() -> false); // TODO: add actual robot state condition

  public ArmDefaultCommand() {
    addRequirements(RobotContainer.getArm());
  }

  private Command onTrueCommand =
      ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  
  private Command onFalseCommand =
      ArmFactory.setPosition(Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  
  @Override
  public void initialize() {
    condition.onTrue(onTrueCommand);
    condition.onFalse(onFalseCommand);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      onTrueCommand.cancel();
      onFalseCommand.cancel();
    }
  }
}
