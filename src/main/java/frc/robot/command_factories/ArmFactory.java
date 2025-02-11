package frc.robot.command_factories;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Factory class for creating commands related to the arm subsystem. */
public class ArmFactory {
  /**
   * Creates a command to set the arm position.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @param position Target position in degrees
   * @return Command to set arm position
   */
  public static Command setPosition(RobotContainer container, double position) {
    return container.getArm().setPosition(position).withName("Set Arm Position")
    .onlyIf(() -> container.getElevator().getRotationCount() > Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS);
  }

  /**
   * Creates a command for manual arm control.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @param power Power level (-1 to 1)
   * @return Command for manual arm control
   */
  public static Command manualControl(RobotContainer container, double power) {
    return container.getArm().manual(new DutyCycleOut(power)).withName("Manual Arm Control")
    .onlyIf(() -> container.getElevator().getRotationCount() > Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS);
  }

  /**
   * Creates a command to stop the arm.
   * NOTE: this may be a nonsense method.  Replace with what makes sense.
   *
   * @param container The RobotContainer instance
   * @return Command to stop arm
   */
  public static Command stop(RobotContainer container) {
    return container.getArm().stop().withName("Stop Arm");
  }
}
