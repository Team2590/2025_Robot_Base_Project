package frc.robot.command_factories;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.Constants.ElevatorConstantsLeonidas;
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
        .setPositionCommand(position)
        .withName("Set Arm Position")
        .onlyIf(
            () ->
                SafetyChecker.isSafe(
                    SafetyChecker.MechanismType.ARM_MOVEMENT, wrapArmSetpoint(position)));
  }

  public static Command setPositionBlocking(double position) {
    return RobotContainer.getArm()
        .setPositionBlocking(position)
        .withName("Set Arm Position Blocking")
        .onlyIf(
            () ->
                SafetyChecker.isSafe(
                    SafetyChecker.MechanismType.ARM_MOVEMENT, wrapArmSetpoint(position)));
  }

  public static Command setPositionRun(double position) {
    return RobotContainer.getArm()
        .setPositionRun(position)
        .withName("Set Arm Position Run")
        .onlyIf(
            () ->
                SafetyChecker.isSafe(
                    SafetyChecker.MechanismType.ARM_MOVEMENT, wrapArmSetpoint(position)));
  }

  //

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

  /*
   * Helper Function, picks the closest value for the arm setpoint based on if the cancoder is wrapped or not
   */
  public static double wrapArmSetpoint(double setpoint) {

    double current = RobotContainer.getArm().getAbsolutePosition();
    double wrappedSetpoint = setpoint + ArmConstantsLeonidas.ARM_WRAP_POS;
    if (RobotContainer.getElevator().getRotationCount()
        >= ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS) {
      return NemesisMathUtil.selectClosest(setpoint, wrappedSetpoint, current);
    } else {
      return setpoint;
    }
  }
}
