package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class EndEffectorFactory {

  /**
   * Creates a command to run the endeffector intake.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runEndEffector() {
    return RobotContainer.getEndEffector().runEndEffector().withName("Run Endeffector Intake");
  }

  /**
   * Creates a command to run the endeffector outtake.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runEndEffectorOuttake() {
    return RobotContainer.getEndEffector().runEndEffectorOuttake().withName("Run Endeffector Outtake");
  }
}
