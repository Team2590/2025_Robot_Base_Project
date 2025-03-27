package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.command_factories.EndEffectorFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class GrabAlgaeCommand extends Command {
    EndEffector endEffector = RobotContainer.getEndEffector();
    Arm arm = RobotContainer.getArm();
    Elevator elevator = RobotContainer.getElevator();
    Level level;

  public GrabAlgaeCommand(Level level) {
    this.level = level;
    addRequirements(RobotContainer.getEndEffector(), RobotContainer.getArm(), RobotContainer.getElevator());
  }

  @Override
  public void initialize(){
    // Clear out Algae State
    RobotState.setEndEffectorNoAlgae();
  }

  @Override
  public void execute() {
    switch (level) {
        case DEALGAE_L2:
        // TODO Call Dhruv's new synchronization command
        arm.setPosition(RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).armSetpoint);
        elevator.setPosition(RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L2).elevatorSetpoint);
        break;
        // TODO Call Dhruv's new synchronization command
        case DEALGAE_L3:
        arm.setPosition(RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L3).armSetpoint);
        elevator.setPosition(RobotState.getInstance().getDealgaeSetpoints(Level.DEALGAE_L3).elevatorSetpoint);
        break;
        default:
        // TODO Call Dhruv's new synchronization command
        arm.setPosition(Constants.ArmConstantsLeonidas.ARM_SET_STOW);
        elevator.setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_STOW_POS);
        break;
      };

    endEffector.runEndEffectorGrabAndHoldAlgae();

  }

  @Override
  public boolean isFinished() {
    // TODO want this to be zone dependent
    return endEffector.hasGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    
    RobotState.setEndEffectorHasAlgae();
    // MUST HAPPEN AFTER
    // Want to set this back to algae stow position, but need to wait on this until the frcpolygon stuff works
  }
}
