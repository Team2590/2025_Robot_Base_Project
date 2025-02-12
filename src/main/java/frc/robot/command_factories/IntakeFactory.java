package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating commands related to the intake subsystem.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
 *
 * <p>This class provides methods to create commands for controlling the intake mechanism.
 */
public class IntakeFactory {
  private static RobotContainer container = Robot.getRobotContainerInstance();
  /**
   * Creates a command to run the intake at a specified speed.
   *
   * @param container The RobotContainer instance
   * @param intakeSpeed Supplier for the intake speed
   * @return Command to run the intake
   */
  public static Command runIntake(DoubleSupplier intakeSpeed) {
    return container.getIntake().runIntake(intakeSpeed.getAsDouble()).withName("Run Intake");
  }

  /**
   * Creates a command to set the intake to the coral position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to coral position
   */
  public static Command setIntakeCoralPosition() {
    return container.getIntake().setIntakeCoralPosition().withName("Set Intake Coral Position");
  }

  /**
   * Creates a command to set the intake to the algae position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command to set intake to algae position
   */
  public static Command setIntakeAlgaePosition() {
    return container.getIntake().setIntakeAlgaePosition().withName("Set Intake Algae Position");
  }
}
