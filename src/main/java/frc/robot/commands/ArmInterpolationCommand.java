package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;
import frc.robot.util.TreeLookup;

public class ArmInterpolationCommand extends Command {
    private TreeLookup lookupTable;
    private double setpoint;

    public ArmInterpolationCommand(TreeLookup lookupTable, double setpoint) {
        this.lookupTable = lookupTable;
        this.setpoint = setpoint;
        addRequirements(RobotContainer.getArm());
    }

    @Override
    public void execute() {
        double setpoint = lookupTable.getValue(RobotContainer.getElevator().getRotationCount());
        RobotContainer.getArm().getIO().setPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return NemesisMathUtil.isApprox(RobotContainer.getArm().getAbsolutePosition(), 0.01, setpoint);
    }
}
