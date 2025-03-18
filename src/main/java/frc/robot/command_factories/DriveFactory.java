package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import java.util.function.Supplier;

/**
 * Factory class for creating commands related to the drive subsystem.
 *
 * <p>This class provides methods to create commands for controlling the robot's movement.
 */
public class DriveFactory {
  /**
   * Creates a command for joystick-controlled driving.
   *
   * @param container The RobotContainer instance
   * @return Command for joystick driving
   */
  public static Command joystickDrive() {

    return DriveCommands.joystickDrive(
            RobotContainer.getDrive(),
            () -> -RobotContainer.getLeftJoystick().getY(),
            () -> -RobotContainer.getLeftJoystick().getX(),
            () -> -RobotContainer.getRightJoystick().getX())
        .withName("Joystick Drive");
  }

  /**
   * Creates a command to drive to a specific pose.
   *
   * @param container The RobotContainer instance
   * @param targetPose The target pose to drive to
   * @return Command to drive to pose
   */
  public static Command driveToPose(Pose2d targetPose) {

    // RobotContainer.factoryCommands.add("Drive To Pose");
    return DriveCommands.driveToPose(targetPose).withName("Drive To Pose");
  }

  /**
   * Creates a command that first uses path following to get close to the target, then uses PID
   * control for final precise alignment.
   */
  public static Command hybridPreciseAlignment(
      Supplier<Pose2d> targetPoseSupplier, Rotation2d approachDirection) {

    return Commands.sequence(
            DriveCommands.preciseAlignment(
                RobotContainer.getDrive(), targetPoseSupplier, approachDirection),
            DriveCommands.pidAlignment(RobotContainer.getDrive(), targetPoseSupplier))
        .withName("HybridPreciseAlignment");
  }

  /**
   * Creates a command for testing the hybrid precise alignment in simulation.
   *
   * @param targetPoseSupplier The supplier for the target pose
   * @param approachDirection The direction to approach the target from
   * @return A command that performs precise alignment using both methods
   */
  public static Command hybridPreciseAlignmentSim(
      Supplier<Pose2d> targetPoseSupplier, Rotation2d approachDirection) {

    return hybridPreciseAlignment(targetPoseSupplier, approachDirection);
  }
}
