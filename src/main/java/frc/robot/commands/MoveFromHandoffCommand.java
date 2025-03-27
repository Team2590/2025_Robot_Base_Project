package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

import java.awt.Robot;
import java.util.function.DoubleSupplier;

public class MoveFromHandoffCommand extends Command {
  private double armSetpoint;
  private double elevatorSetpoint;
  private double intakeArmSetpoint;
  private DoubleSupplier armThreshold;

  /**
   * @param armThreshold - max arm rotation count to achieve before being able to move in parallel
   */
  public MoveFromHandoffCommand(
      DoubleSupplier armThreshold,
      double armTargetPos,
      double elevatorTargetPos,
      double intakeTargetPos) {
    setName("Move to handoff");
    this.armThreshold = armThreshold;
    this.armSetpoint = armTargetPos;
    this.elevatorSetpoint = elevatorTargetPos;
    this.intakeArmSetpoint = intakeTargetPos;
    addRequirements(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake());
  }

  /** Defaulting to stow */
  public MoveFromHandoffCommand() {
    this(
        () -> Constants.ArmConstantsLeonidas.ARM_THRESHOLD_POS,
        Constants.ArmConstantsLeonidas.ARM_SET_STOW,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS,
        Constants.IntakeArmConstantsLeonidas.INTAKE_HOME_POS);
  }

  @Override
  public void execute() {
    if (RobotContainer.getArm().getAbsolutePosition() > armThreshold.getAsDouble()) {
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
