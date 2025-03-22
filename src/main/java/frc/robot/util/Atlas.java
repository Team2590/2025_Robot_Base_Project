package frc.robot.util;

import java.awt.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

/**
 * Atlas is a utility class that synchronizes the movement of the intake, elevator, and arm to optimize speed.
 * Named after the Greek Titan, associated with mapmaking
 */
public class Atlas {
    /**
     * Sets intake, elevator, and arm to a particular position, synchronizing its movement to optimize speed. 
     * @param intakeTargetPos - target position of the intake
     * @param elevatorTargetPos - target position of the elevator
     * @param armTargetPos - target position of the arm
     * @return a Command that optimizes the movement of the subsystems to the desired positions
     */

    public static Command synchronize(double intakeTargetPos, double elevatorTargetPos, double armTargetPos) {
        Elevator elevator = RobotContainer.getElevator();
        Arm arm = RobotContainer.getArm();
        Intake intake = RobotContainer.getIntake();
        double handoffPos = Constants.ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS;
        if (RobotContainer.getElevator().getRotationCount() >= handoffPos &&
            elevatorTargetPos >= handoffPos) {
                return new ParallelCommandGroup(
                    arm.setPositionBlocking(armTargetPos),
                    elevator.setPositionBlocking(elevatorTargetPos),
                    intake.setPositionBlocking(intakeTargetPos));
            }
        else if (elevator.getRotationCount() >= handoffPos && 
        elevatorTargetPos < handoffPos){
            return new SequentialCommandGroup(arm.setPositionBlocking(armTargetPos), 
                                                intake.setPositionBlocking(intakeTargetPos), 
                                                elevator.setPositionBlocking(elevatorTargetPos));

        }
        else if(elevator.getRotationCount() < handoffPos && elevatorTargetPos >= handoffPos){
            return new SequentialCommandGroup(elevator.setPositionBlocking(elevatorTargetPos), new ParallelCommandGroup(arm.setPositionBlocking(armTargetPos), intake.setPositionBlocking(intakeTargetPos)));
        }
        else if (elevator.getRotationCount() < handoffPos && elevatorTargetPos < handoffPos){
            return new SequentialCommandGroup(intake.setPositionBlocking(intakeTargetPos), new ParallelCommandGroup(arm.setPositionBlocking(armTargetPos), elevator.setPositionBlocking(elevatorTargetPos)));
        }
        else{
            System.out.println("Magical state where somehow none of the above is true. You definitely messed something up. Bad programmer. BAD!");
            return Commands.none();
        }
    }
}
