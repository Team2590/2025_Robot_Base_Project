package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;
import java.util.function.DoubleSupplier;

public class MoveToHandoffCommand extends Command {
  private double armSetpoint;
  private double elevatorSetpoint;
  private double intakeArmSetpoint;
  private DoubleSupplier elevatorThreshold;

  /**
   * @param elevatorThreshold - minimum elevator rotation count before moving the elevator and arm
   *     in parallel
   */
  public MoveToHandoffCommand(DoubleSupplier elevatorThreshold) {
    setName("Move to handoff");
    this.elevatorThreshold = elevatorThreshold;
    this.armSetpoint = Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS;
    this.elevatorSetpoint = Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_TRANSITION_POS;
    this.intakeArmSetpoint = Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS;
    addRequirements(RobotContainer.getElevator(), RobotContainer.getArm());
  }

  /** Default constructor, sets the elevator threshold to 0 */
  public MoveToHandoffCommand() {
    this(() -> 0);
  }

  @Override
  public void execute() {
    if (RobotContainer.getElevator().getRotationCount() < elevatorThreshold.getAsDouble()) {
      RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
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
