package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstantsLeonidas;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>This class provides methods to create commands for controlling the intake mechanism.
 */
public class IntakeFactory {
  /**
   * Creates a command to run the intake at a specified speed.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runIntake(DoubleSupplier intakeSpeed) {
    return RobotContainer.getIntake()
        .runIntakeUntilHasCoral(intakeSpeed.getAsDouble())
        .withName("Run Intake");
  }

  public static Command runIntakeVoltage(DoubleSupplier speed) {
    return RobotContainer.getIntake()
        .runIntakeVoltage(speed.getAsDouble())
        .withName("Run Intake Voltage");
  }

  /**
   * Creates a command to set the intake to the coral position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to coral position
   */
  public static Command setIntakeCoralPosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS)
        .withName("Set Intake Coral Position");
  }

  public static Command setPosition(double position) {
    return RobotContainer.getIntake().setPosition(position).withName("Set Position");
  }

  public static Command setPositionBlocking(double position) {
    return RobotContainer.getIntake().setPositionBlocking(position).withName("Set Position");
  }
  /**
   * Creates a command to set the intake to the algae position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to algae position
   */
  public static Command setIntakeAlgaePosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS)
        .withName("Set Intake Algae Position");
  }

  /**
   * Creates a command to set the intake to the home position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to algae position
   */
  public static Command setHomePosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS)
        .withName("Set Intake Home Position");
  }

  /**
   * Creates a command to set the intake to the home position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to algae position
   */
  public static Command setHoldingAlgaePosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeArmConstantsLeonidas.INTAKE_GROUND_CORAL_POS)
        .withName("Set Intake Home Position");
  }
}
