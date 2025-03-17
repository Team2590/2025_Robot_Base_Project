package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class IntakeDefaultCommand {
    private Command hasCoralCommand =
      Commands.waitSeconds(0.1)
          .andThen(IntakeFactory.setPosition(Constants.IntakeArmConstantsLeonidas.INTAKE_CORAL_POS))
          .withName("Intake has coral default command");

    public IntakeDefaultCommand() {
        addRequirements(RobotContainer.getIntake());
    }

      @Override
  public void execute() {
    if (RobotState.endEffectorhasCoral()) hasCoralCommand.schedule();
    else hasCoralCommand.cancel();
  }

  @Override
  public void end(boolean interrupted) {
    hasCoralCommand.cancel();
  }
}