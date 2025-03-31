package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;

// spotless:off
public class MoveToHandoffCommand extends Command {
  private double armSetpoint;
  private double elevatorSetpoint;
  private double intakeArmSetpoint;

  public MoveToHandoffCommand() {
    setName("Move to handoff");
    this.armSetpoint = Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS;
    this.elevatorSetpoint = Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_TRANSITION_POS;
    this.intakeArmSetpoint = Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS;
    addRequirements(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake());
  }

  @Override
  public void execute() {
    if (RobotContainer.getElevator().getRotationCount() < Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS && RobotContainer.getArm().getAbsolutePosition() > Constants.ArmConstantsLeonidas.ARM_VERTICAL_POS + 0.05) {
      RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
    } else {
      if (!RobotContainer.getIntake().hasCoral()){
        RobotContainer.getIntake().getArmIO().setPosition(intakeArmSetpoint);
      }
      RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
      RobotContainer.getArm().getIO().setPosition(armSetpoint);
    }
  }

  @Override
  public boolean isFinished() {
    boolean intakeAtsetpoint;
    if (RobotContainer.getIntake().hasCoral()){
      intakeAtsetpoint = true;
    }else{
      intakeAtsetpoint = NemesisMathUtil.isApprox(RobotContainer.getIntake().getArmRotationCount(), 0.05, intakeArmSetpoint);
    }
    return
      NemesisMathUtil.isApprox(RobotContainer.getElevator().getRotationCount(), 0.05, elevatorSetpoint) &&
      NemesisMathUtil.isApprox(RobotContainer.getArm().getAbsolutePosition(), 0.01, armSetpoint) &&
      intakeAtsetpoint;       
  }
}
// spotless:on
