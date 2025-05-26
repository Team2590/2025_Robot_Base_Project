package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;
import org.littletonrobotics.junction.Logger;

// spotless:off

public class MoveToHandoffCommand extends Command {
  private final double targetArmSetpoint;
  private final double elevatorSetpoint;
  private final double intakeArmSetpoint;
  private final double armStowSetpoint;

  LoggedTunableNumber minElevatorHeight = new LoggedTunableNumber("Handoff/MinElevatorHeight", Constants.ElevatorConstantsLeonidas.MIN_ELEVATOR_HEIGHT_ARM_PARALLEL);

  public MoveToHandoffCommand() {
    setName("Move to Handoff");

    this.targetArmSetpoint = Constants.ArmConstantsLeonidas.ARM_HANDOFF_POS;
    this.elevatorSetpoint = Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_TRANSITION_POS;
    this.intakeArmSetpoint = Constants.IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS;
    this.armStowSetpoint = Constants.ArmConstantsLeonidas.ARM_SET_STOW;

    // Declare subsystem requirements
    addRequirements(RobotContainer.getElevator(), RobotContainer.getArm(), RobotContainer.getIntake());
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.getElevator().setPosition(this.elevatorSetpoint);

    if (!RobotContainer.getIntake().hasCoral()) {
      RobotContainer.getIntake().getArmIO().setPosition(this.intakeArmSetpoint);
    }

    double currentElevatorPos = RobotContainer.getElevator().getRotationCount();
    double elevatorThreshold = minElevatorHeight.get();

    if (currentElevatorPos < elevatorThreshold) {
      RobotContainer.getArm().getIO().setPosition(RobotState.getInstance().getStowSetpoint());
    } else {
      RobotContainer.getArm().getIO().setPosition(this.targetArmSetpoint);
    }
  }

  @Override
  public boolean isFinished() {
    boolean intakeAtSetpoint;
    if (RobotContainer.getIntake().hasCoral()) {
      intakeAtSetpoint = true;
    } else {
      intakeAtSetpoint = NemesisMathUtil.isApprox(
          RobotContainer.getIntake().getArmRotationCount(),
          0.15,
          this.intakeArmSetpoint);
    }

    boolean elevatorAtSetpoint = NemesisMathUtil.isApprox(
        RobotContainer.getElevator().getRotationCount(),
        0.05,
        this.elevatorSetpoint);

    boolean armAtSetpoint = NemesisMathUtil.isApprox(
        RobotContainer.getArm().getAbsolutePosition(),
        0.05,
        this.targetArmSetpoint);
    
      Logger.recordOutput("MoveToHandoff/armAtsetpoint", armAtSetpoint);
      Logger.recordOutput("MoveToHandoff/elevatorAtSetpoint", elevatorAtSetpoint);
      Logger.recordOutput("MoveToHandoff/intakeAtSetpoint", intakeAtSetpoint);

    return elevatorAtSetpoint && armAtSetpoint && intakeAtSetpoint;
  }

  @Override
    public void end(boolean interrupted) {
    }
}
// spotless:on
