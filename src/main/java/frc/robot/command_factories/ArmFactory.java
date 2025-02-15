package frc.robot.command_factories;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Factory class for creating commands related to the arm subsystem. */
public class ArmFactory {
  private static RobotContainer container = Robot.getRobotContainerInstance();
  /**
   * Creates a command to set the arm position.
   *
   * @param container The RobotContainer instance
   * @param position Target position in degrees
   * @return Command to set arm position
   */
  public static Command setPosition(double position) {
    return container
        .getArm()
        .setPosition(position)
        .withName("Set Arm Position")
        .onlyIf(
            () ->
                container.getElevator().getRotationCount()
                    > Constants.ElevatorConstantsLeonidas.ARM_FACTORY_MIN_POS);
  }

  public static Command setPositionBlocking(double position) {
    return container
        .getArm()
        .setPositionBlocking(position)
        .withName("Set Arm Position Blocking")
        .onlyIf(
            () ->
                container.getElevator().getRotationCount()
                    > Constants.ElevatorConstantsLeonidas.ARM_FACTORY_MIN_POS);
  }

  /**
   * Creates a command for manual arm control.
   *
   * @param container The RobotContainer instance
   * @param power Power level (-1 to 1)
   * @return Command for manual arm control
   */
  public static Command manualControl(double power) {
    return container
        .getArm()
        .manual(new DutyCycleOut(power))
        .withName("Manual Arm Control")
        .onlyIf(
            () ->
                container.getElevator().getRotationCount()
                    > Constants.ElevatorConstantsLeonidas.ARM_FACTORY_MIN_POS);
  }

  /**
   * Creates a command to stop the arm.
   *
   * @param container The RobotContainer instance
   * @return Command to stop arm
   */
  public static Command stop() {
    return container.getArm().stop().withName("Stop Arm");
  }
}
