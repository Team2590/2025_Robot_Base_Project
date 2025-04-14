package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class EndEffectorFactory {

  /**
   * Creates a command to run the endeffector intake.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runEndEffector() {
    return RobotContainer.getEndEffector()
        .runEndEffectorIntake()
        // .raceWith(LEDFactory.blink())
        // .andThen(LEDFactory.solid())
        .withName("Run Endeffector Intake");
  }

  public static Command runEndEffectorVoltage(double voltage) {
    return RobotContainer.getEndEffector().runEndEffectorVoltage(voltage);
  }

  public static Command runEndEffectorManual() {
    return RobotContainer.getEndEffector().runEndEffectorManual();
  }

  public static Command runEndEffectorGrabAndHoldAlgae() {
    return RobotContainer.getEndEffector()
        .runEndEffectorVoltage(Constants.EndEffectorConstantsLeonidas.INTAKE_ALGAE_VOLTAGE)
        .until(() -> RobotState.endEffectorHasGamePiece())
        .andThen(Commands.runOnce(() -> RobotState.getInstance().setHasAlgae(true)));
  }
  /**
   * Creates a command to run the endeffector outtake.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runEndEffectorOuttake() {
    return RobotContainer.getEndEffector()
        .runEndEffectorOuttake()
        // .raceWith(LEDFactory.blink())
        // .andThen(LEDFactory.solid())
        .withName("Run Endeffector Outtake");
  }

  public static boolean endEffectorCommandFinished() {
    return !RobotContainer.getEndEffector().isRunning();
  }
}
