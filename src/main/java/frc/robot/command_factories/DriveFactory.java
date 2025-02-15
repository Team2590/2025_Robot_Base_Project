package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;

/**
 * Factory class for creating commands related to the drive subsystem.
 *
 * <p>This class provides methods to create commands for controlling the robot's movement.
 */
public class DriveFactory {
  private static RobotContainer container = Robot.getRobotContainerInstance();
  /**
   * Creates a command for joystick-controlled driving.
   *
   * @param container The RobotContainer instance
   * @return Command for joystick driving
   */
  public static Command joystickDrive() {
    return DriveCommands.joystickDrive(
            container.getDrive(),
            () -> -container.getLeftJoystick().getY(),
            () -> -container.getLeftJoystick().getX(),
            () -> -container.getRightJoystick().getX())
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

    //RobotContainer.factoryCommands.add("Drive To Pose");
    return DriveCommands.driveToPose(targetPose).withName("Drive To Pose");

 
 
  }
  
}
