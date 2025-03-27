package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NemesisMathUtil;

public class MoveArmAndElevatorCommand extends Command {
    private double armSetpoint;
    private double elevatorSetpoint;
    private LoggedTunableNumber ELEVATOR_THRESHOLD_POS = new LoggedTunableNumber("Move command", 0);
    private LoggedTunableNumber INTAKE_SAFE_STOW_POS = new LoggedTunableNumber("Move command", 0);

    public MoveArmAndElevatorCommand(double armSetpoint, double elevatorSetpoint) {
        this.armSetpoint = armSetpoint;
        this.elevatorSetpoint = elevatorSetpoint;
    }

    @Override
    public void execute() {
        if (RobotContainer.getIntake().getArmRotationCount() < INTAKE_SAFE_STOW_POS.get()) {
            RobotContainer.getIntake().getArmIO().setPosition(INTAKE_SAFE_STOW_POS.get());
        } else {
            if (RobotContainer.getElevator().getRotationCount() < ELEVATOR_THRESHOLD_POS.get() && RobotContainer.getArm().getAbsolutePosition() > 0.5) {
                RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
            } else {
                RobotContainer.getElevator().getIO().setPosition(elevatorSetpoint);
                RobotContainer.getArm().getIO().setPosition(armSetpoint);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return NemesisMathUtil.isApprox(RobotContainer.getElevator().getRotationCount(), 0.05, elevatorSetpoint) && NemesisMathUtil.isApprox(RobotContainer.getArm().getAbsolutePosition(), 0.01, armSetpoint);
    }
}
