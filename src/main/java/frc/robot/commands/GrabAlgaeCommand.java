package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.util.NemesisMathUtil;

public class GrabAlgaeCommand extends MoveFromHandoffCommand {
  private final EndEffector endEffector;
  private final Arm arm;
  private final Elevator elevator;
  private final Level level;
  private final double elevatorSetpoint;
  private final double armSetpoint;
  private final double intakeArmSetpoint;

  public GrabAlgaeCommand(
      Level level,
      EndEffector endEffector,
      Arm arm,
      Elevator elevator,
      double elevatorSetpoint,
      double armSetpoint,
      double intakeArmSetpoint) {
    super(intakeArmSetpoint, elevatorSetpoint, armSetpoint);
    setName("GrabAlgae");
    this.level = level;
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;
    this.intakeArmSetpoint = intakeArmSetpoint;
    this.endEffector = endEffector;
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(this.endEffector, this.arm, this.elevator);
  }

  @Override
  public boolean isFinished() {
    return endEffector.hasGamePiece()
        && NemesisMathUtil.isApprox(elevator.getRotationCount(), 0.05, elevatorSetpoint)
        && NemesisMathUtil.isApprox(arm.getAbsolutePosition(), 0.01, armSetpoint)
        && NemesisMathUtil.isApprox(
            RobotContainer.getIntake().getArmRotationCount(), 0.05, intakeArmSetpoint);
  }

  @Override
  public void end(boolean interrupted) {

    // MUST HAPPEN AFTER
    // Want to set this back to algae stow position, but need to wait on this until the frcpolygon
    // stuff works
    if (interrupted) {
      System.out.println("GrabAlgaeCommand was interrupted!");
      // Optionally, you could add some logic to stop the mechanisms here if interrupted.
    } else {
      System.out.println("GrabAlgaeCommand finished successfully!");
    }
    elevator.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS);
    arm.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS);
  }
}
