package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;

/**
 * Factory class for creating commands related to the drive subsystem.
 *
 * <p>This class provides methods to create commands for controlling the robot's movement.
 */
public class DriveFactory {
  /**
   * Creates a command for joystick-controlled driving.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command for joystick driving
   */
  public static Command joystickDrive(RobotContainer container) {
    return DriveCommands.joystickDrive(
            container.getDrive(),
            () -> -container.getLeftJoystick().getY(),
            () -> -container.getLeftJoystick().getX(),
            () -> -container.getRightJoystick().getX())
        .withName("Joystick Drive");
  }

  /**
   * Creates a command to drive to a specific pose.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @param targetPose The target pose to drive to
   * @return Command to drive to pose
   */
  public static Command driveToPose(RobotContainer container, Pose2d targetPose) {
    return DriveCommands.driveToPose(targetPose).withName("Drive To Pose");
  }
}
