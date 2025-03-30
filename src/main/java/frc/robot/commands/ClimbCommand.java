package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ClimbFactory;
import frc.robot.command_factories.LEDFactory;
import frc.robot.util.NemesisMathUtil;
import java.util.Set;

public class ClimbCommand extends Command {

  private Trigger limitSwitch = new Trigger(() -> RobotContainer.getClimb().getLimitSwitchValue());
  private Command deployMech =
      Commands.sequence(
          new MoveFromHandoffCommand(
              Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
              .33,
              Constants.ArmConstantsLeonidas.ARM_SET_STOW),
          ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION));

  // return new MoveFromHandoffCommand(
  //     IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
  //     .33,
  //     Constants.ArmConstantsLeonidas.ARM_SET_STOW)
  // .andThen(ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MECHANISM_POSITION))
  // .andThen(
  //     // ArmFactory.setPositionBlocking(Constants.ArmConstantsLeonidas.CLIMB_POS),
  //     // ElevatorFactory.setPositionBlocking(Constants.ElevatorConstantsLeonidas.CLIMB_POS),
  //     ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION)
  //         .onlyWhile(() -> RobotContainer.getClimb().getLimitSwitchValue()))
  // // .andThen(LEDFactory.auraRizz())
  // .withName("Climb");

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
  public void execute() {
    // Set subsystems in correct positions
    deployMech.schedule();
    if (deployMech.isFinished()) {
      limitSwitch.onTrue(
        Commands.parallel(
          ClimbFactory.runClimb(Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION),
          LEDFactory.auraRizz()));
    }
  }

  @Override
  public boolean isFinished() {
    return NemesisMathUtil.isApprox(
        RobotContainer.getClimb().getRotationCount(),
        1,
        Constants.ClimbConstantsLeonidas.CLIMB_MAX_POSITION);
  }
}
