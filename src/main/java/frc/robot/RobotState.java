package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

/**
 * A singleton class that holds references to all of the robot's subsystems. This class is used to
 * pass around the robot's state to various components of the robot.
 */
public class RobotState {

  private final Arm arm;
  private final Drive drive;
  private final Elevator elevator;
  private final Vision vision;
  private final EndEffector endEffector;
  private final Intake intake;

  private static RobotState instance;

  private RobotState(
      Arm arm,
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Vision vision) {
    this.arm = arm;
    this.drive = drive;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.intake = intake;
    this.vision = vision;
  }

  public static RobotState initialize(
      Arm arm,
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      Intake intake,
      Vision vision) {
    if (instance != null) {
      throw new IllegalStateException("RobotState has already been initialized");
    }
    instance = new RobotState(arm, drive, elevator, endEffector, intake, vision);
    return instance;
  }

  /**
   * Returns the singleton instance. This method should only be called after initialize has been
   * called.
   */
  public static RobotState getInstance() {
    if (instance == null) {
      throw new IllegalStateException("RobotState has not been initialized");
    }
    return instance;
  }

  /** Returns the current pose of the robot. */
  public Pose2d getPose() {
    return drive.getPose();
  }
}
