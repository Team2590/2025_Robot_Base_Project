package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstantsLeonidas;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.util.NemesisMathUtil;
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

  public static Command setPosition(double position) {
    return RobotContainer.getIntake().setPosition(position).withName("Set Position");
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

  public static Command defaultCommand() {
    return RobotContainer.getIntake()
        .setPosition(Constants.IntakeConstantsLeonidas.INTAKE_FACTORY_HOME_POSITION)
        .onlyIf(
            () ->
                !NemesisMathUtil.isApprox(
                        RobotContainer.getArm().getAbsolutePosition(),
                        0.05,
                        Constants.IntakeConstantsLeonidas.INTAKE_FACTORY_HOME_POSITION)
                    && !RobotState.intakeHasCoral()
                    && !RobotState.intakeHasAlgae())
        .withName("Intake default command")
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .finallyDo(() -> System.out.println("Intake default command finished"));
  }
}
