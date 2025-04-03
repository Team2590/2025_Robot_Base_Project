package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

public class MoveFromHandoffCommand extends Command {
  private double armSetpoint;
  private double elevatorSetpoint;
  private double intakeArmSetpoint;
  private double armThresholdFront;
  private double armThresholdBack;

  /**
   * @param armThreshold - max arm rotation count to achieve before being able to move in parallel
   */
  public MoveFromHandoffCommand(
      double intakeTargetPos, double elevatorTargetPos, double armTargetPos) {
    setName("Move to handoff");
    this.armThresholdFront = Constants.ArmConstantsLeonidas.ARM_THRESHOLD_POS;
    this.armThresholdBack = Constants.ArmConstantsLeonidas.ARM_THRESHOLD_POS - Constants.ArmConstantsLeonidas.ARM_SET_STOW;
    this.armSetpoint = armTargetPos;
    this.elevatorSetpoint = elevatorTargetPos;
    this.intakeArmSetpoint = intakeTargetPos;
    addRequirements(
        RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake());
  }

  /** Defaulting to stow */
  public MoveFromHandoffCommand() {
    this(
        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW);
  }

  @Override
  public void execute() {

    if (this.armSetpoint >= RobotContainer.getArm().getAbsolutePosition()){  // scoring front
      if (RobotContainer.getArm().getAbsolutePosition() <= armThresholdFront
          && elevatorSetpoint < Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
          RobotContainer.getArm().getIO().setPosition(armSetpoint);
      } else {
          RobotContainer.getIntake().getArmIO().setPosition(intakeArmSetpoint);
          RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
          RobotContainer.getArm().getIO().setPosition(armSetpoint);
      }
    }else{
      if (RobotContainer.getArm().getAbsolutePosition() >= armThresholdBack
          && elevatorSetpoint < Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
          RobotContainer.getArm().getIO().setPosition(armSetpoint);
      } else {
          RobotContainer.getIntake().getArmIO().setPosition(intakeArmSetpoint);
          RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
          RobotContainer.getArm().getIO().setPosition(armSetpoint);
      }
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
