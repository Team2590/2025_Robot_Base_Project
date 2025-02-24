package frc.robot.command_factories;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisMathUtil;
import frc.robot.util.SafetyChecker;

/** Factory class for creating commands related to the arm subsystem. */
public class ArmFactory {
  /**
   * Creates a command to set the arm position.
   *
   * @param container The RobotContainer instance
   * @param position Target position in degrees
   * @return Command to set arm position
   */
  public static Command setPosition(double position) {
    return RobotContainer.getArm()
        .setPosition(position)
        .withName("Set Arm Position")
        .onlyIf(() -> SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, position));
  }

  public static Command setPositionBlocking(double position) {
    return RobotContainer.getArm()
        .setPositionBlocking(position)
        .withName("Set Arm Position Blocking")
        .onlyIf(() -> SafetyChecker.isSafe(SafetyChecker.MechanismType.ARM_MOVEMENT, position));
  }

  public static Command defaultCommand() {
    return RobotContainer.getArm()
        .setPosition(Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS)
        .onlyIf(
            () ->
                !NemesisMathUtil.isApprox(
                    RobotContainer.getArm().getAbsolutePosition(),
                    0.05,
                    Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION))
        .withName("Arm default command")
        .finallyDo(() -> System.out.println("Arm default command finished"));
  }

  /**
   * Creates a command for manual arm control.
   *
   * @param container The RobotContainer instance
   * @param power Power level (-1 to 1)
   * @return Command for manual arm control
   */
  public static Command manualControl(double power) {
    return RobotContainer.getArm()
        .manual(new DutyCycleOut(power))
        .withName("Manual Arm Control")
        .onlyIf(
            () ->
                RobotContainer.getElevator().getRotationCount()
                    > Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS);
  }

  /**
   * Creates a command to stop the arm.
   *
   * @param container The RobotContainer instance
   * @return Command to stop arm
   */
  public static Command stop() {
    return RobotContainer.getArm().stop().withName("Stop Arm");
  }
}
