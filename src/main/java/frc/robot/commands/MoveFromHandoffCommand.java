package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

public class MoveFromHandoffCommand extends Command {
  private double armSetpoint;
  private double elevatorSetpoint;
  private double intakeArmSetpoint;
  private double armFrontThreshold;
  private double armBackThreshold;
  private double SETPOINT_TOLERANCE = 0.05;

  public MoveFromHandoffCommand(
      double intakeTargetPos, double elevatorTargetPos, double armTargetPos) {
    setName("Move to handoff");
    this.armFrontThreshold = Constants.ArmConstantsLeonidas.ARM_BACK_THRESHOLD_POS;
    this.armBackThreshold = Constants.ArmConstantsLeonidas.ARM_FRONT_THRESHOLD_POS;
    this.armSetpoint = armTargetPos;
    this.elevatorSetpoint = elevatorTargetPos;
    this.intakeArmSetpoint = intakeTargetPos;
    addRequirements(
        RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake());
  }

  /** Defaulting to stow */
  public MoveFromHandoffCommand() {
    this(
        Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW);
  }

  @Override
  public void execute() {
    boolean armApproxAtSetpoint = NemesisMathUtil.isApprox(RobotContainer.getArm().getAbsolutePosition(), SETPOINT_TOLERANCE, armSetpoint);
    boolean armInDangerRange = RobotContainer.getArm().getAbsolutePosition() > armFrontThreshold && RobotContainer.getArm().getAbsolutePosition() < armBackThreshold;
    if ((!armApproxAtSetpoint || armInDangerRange) && elevatorSetpoint < Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
      RobotContainer.getArm().getIO().setPosition(armSetpoint);
    } else {
      RobotContainer.getIntake().getArmIO().setPosition(intakeArmSetpoint);
      RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
      RobotContainer.getArm().getIO().setPosition(armSetpoint);
    }
  }

  @Override
  public boolean isFinished() {
    return NemesisMathUtil.isApprox(
            RobotContainer.getElevator().getRotationCount(), 0.05, elevatorSetpoint)
        && NemesisMathUtil.isApprox(
            RobotContainer.getArm().getAbsolutePosition(), 0.01, armSetpoint)
        && NemesisMathUtil.isApprox(
            RobotContainer.getIntake().getArmRotationCount(), 0.05, intakeArmSetpoint);
  }
}
