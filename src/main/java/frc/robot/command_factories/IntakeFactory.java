package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstantsLeonidas;
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
    return RobotContainer.getIntake().runIntake(intakeSpeed.getAsDouble()).withName("Run Intake");
  }

  /**
   * Creates a command to set the intake to the coral position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to coral position
   */
  public static Command setIntakeCoralPosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeConstantsLeonidas.INTAKE_FACTORY_CORAL_POSITION)
        .withName("Set Intake Coral Position");
  }

  /**
   * Creates a command to set the intake to the algae position.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to algae position
   */
  public static Command setIntakeAlgaePosition() {
    return RobotContainer.getIntake()
        .setPosition(IntakeConstantsLeonidas.INTAKE_FACTORY_ALGAE_POSITION)
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
        .setPosition(IntakeConstantsLeonidas.INTAKE_FACTORY_HOME_POSITION)
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
        .setPosition(IntakeConstantsLeonidas.INTAKE_FACTORY_HOLDING_ALGAE_POSITION)
        .withName("Set Intake Home Position");
  }
}
