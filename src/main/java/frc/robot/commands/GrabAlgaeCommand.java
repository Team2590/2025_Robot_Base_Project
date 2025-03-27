package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.util.NemesisMathUtil;

public class GrabAlgaeCommand extends MoveFromHandoffCommand {
  EndEffector endEffector = RobotContainer.getEndEffector();
  Arm arm = RobotContainer.getArm();
  Elevator elevator = RobotContainer.getElevator();
  Level level;
  double elevatorSetpoint;
  double armSetpoint;
  double intakeArmSetpoint;

  public GrabAlgaeCommand(Level level, EndEffector endEffector, Arm arm, Elevator elevator, double elevatorSetpoint, double armSetpoint, double intakeArmSetpoint) {
    this.level = level;
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;
    this.intakeArmSetpoint = intakeArmSetpoint;
    addRequirements(endEffector, arm, elevator);
  }

  @Override
  public boolean isFinished() {
    // TODO want this to be zone dependent
    return endEffector.hasGamePiece()
        && NemesisMathUtil.isApprox(
            RobotContainer.getElevator().getRotationCount(), 0.05, elevatorSetpoint)
        && NemesisMathUtil.isApprox(
            RobotContainer.getArm().getAbsolutePosition(), 0.01, armSetpoint)
        && NemesisMathUtil.isApprox(
            RobotContainer.getIntake().getArmRotationCount(), 0.05, intakeArmSetpoint);
  }

  @Override
  public void end(boolean interrupted) {

    // MUST HAPPEN AFTER
    // Want to set this back to algae stow position, but need to wait on this until the frcpolygon
    // stuff works
    elevator.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS);
    arm.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS);
  }
}
