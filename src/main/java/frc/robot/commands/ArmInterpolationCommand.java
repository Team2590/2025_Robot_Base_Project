package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;
import frc.robot.util.TreeLookup;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class ArmInterpolationCommand extends Command {
  private TreeLookup lookupTable;
  private double setpoint;

  public ArmInterpolationCommand(
      TreeMap<Double, Double> setpointMap, double elevatorSetpoint, double armSetpoint) {
    this.setpoint = armSetpoint;
    TreeMap<Double, Double> temp = new TreeMap<>(setpointMap);
    temp.put(elevatorSetpoint, armSetpoint);
    this.lookupTable = new TreeLookup(temp);
    addRequirements(RobotContainer.getArm());
  }

  @Override
  public void execute() {
    double intermediateSetpoint =
        lookupTable.getValue(RobotContainer.getElevator().getRotationCount());
    RobotContainer.getArm().getIO().setPosition(intermediateSetpoint);
    Logger.recordOutput("ArmInterpolation/ElevatorPosition", intermediateSetpoint);
    Logger.recordOutput("ArmInterpolationCommand/armSetpoint", intermediateSetpoint);
  }

  @Override
  public boolean isFinished() {
    return NemesisMathUtil.isApprox(RobotContainer.getArm().getAbsolutePosition(), 0.01, setpoint);
  }
}
