package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ElevatorFactory;

public class ElevatorDefaultCommand extends Command {
  private static Trigger condition = new Trigger(() -> false); // TODO: add actual robot state condition

  public ElevatorDefaultCommand() {
    addRequirements(RobotContainer.getElevator());
  }

  private Command onTrueCommand =
      ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  
  private Command onFalseCommand =
      ElevatorFactory.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS)
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
