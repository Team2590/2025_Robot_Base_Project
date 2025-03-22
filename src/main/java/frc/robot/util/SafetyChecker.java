package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstantsLeonidas;
import frc.robot.Constants.ElevatorConstantsLeonidas;

public class SafetyChecker {
  public enum MechanismType {
    ARM_MOVEMENT,
    ELEVATOR_MOVEMENT,
    INTAKE_MOVEMENT
  }

  /**
   * Checks if a mechanism movement is safe based on the positions of both the arm and elevator.
   *
   * <p>The safety rules are as follows:
   *
   * <ul>
   *   <li>For ARM_MOVEMENT: Movement is safe if the arm is within its operational range as defined
   *       by its constants.
   *   <li>For ELEVATOR_MOVEMENT: Movement is safe if the elevator is within its operational range
   *       as defined by its constants.
   * </ul>
   *
   * @param checkType The type of mechanism movement to check (ARM_MOVEMENT or ELEVATOR_MOVEMENT)
   * @param movingPosition The position of the mechanism that is moving
   * @return true if the movement is safe, false otherwise
   */
  public static boolean isSafe(MechanismType checkType, double movingPosition) {
    switch (checkType) {
      case ARM_MOVEMENT:
        // For ARM_MOVEMENT:
        // movingPosition is arm position
        // otherPosition is elevator position
        if (!isArmInOperationalRange(movingPosition)) {
          System.out.println(
              "Arm is out of operational range, movingPosition: "
                  + movingPosition
                  + " operational range: "
                  + Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS
                  + " to "
                  + Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS);
          return false; // trying to overshoot the arm position's capabilities
        }

        return true;

      case ELEVATOR_MOVEMENT:
        // For ELEVATOR_MOVEMENT:
        // movingPosition is elevator position
        // otherPosition is arm position
        if (!isElevatorInOperationalRange(movingPosition)) {
          System.out.println(
              "Elevator is out of operational range, movingPosition: "
                  + movingPosition
                  + " operational range: "
                  + Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                  + " to "
                  + Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS);
          return false; // outside elevators capabilities
        }

        return true;

      case INTAKE_MOVEMENT:
        if (!isIntakeInOperationalRange(movingPosition)) {
          System.out.println(
              "Intake is out of operational range, movingPosition: "
                  + movingPosition
                  + " operational range: "
                  + Constants.IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MIN_POS
                  + " to "
                  + Constants.IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MAX_POS);
          return false;
        }
        return true;

      default:
        return false;
    }
  }

  /*
   * Ensuring subystems are in operational ranges and there is no chance for bad values to be set
   * Assert the Elevator Setpoint and Arm won't both be in danger zone (area where they can collide if not moved sequentially)
   *
   *
   */
  public static boolean operationalSafety(
      double intakeSetpoint, double armSetpoint, double elevatorSetpoint) {
    boolean subsystemsOperational =
        isSafe(MechanismType.ARM_MOVEMENT, armSetpoint)
            && isSafe(MechanismType.INTAKE_MOVEMENT, intakeSetpoint)
            && isSafe(MechanismType.ELEVATOR_MOVEMENT, elevatorSetpoint);
    if (!elevatorOperational(elevatorSetpoint, armSetpoint)) {
      System.out.println(" \n \n ELEVATOR AND ARM BELOW HAND OFF POS (DANGER) \n \n");
    }
    return subsystemsOperational && elevatorOperational(elevatorSetpoint, armSetpoint);
  }

  public static boolean elevatorOperational(double elveatorSetpoint, double armSetpoint) {

    if (elveatorSetpoint < ElevatorConstantsLeonidas.ELEVATOR_HANDOFF_POS
        && armSetpoint < ArmConstantsLeonidas.ARM_HANDOFF_POS) {
      return false;
    }

    return true;
  }

  private static boolean isIntakeInOperationalRange(double intakePosition) {
    return NemesisMathUtil.isBetweenInclusive(
        intakePosition,
        Constants.IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MIN_POS,
        Constants.IntakeArmConstantsLeonidas.INTAKE_OPERATIONAL_MAX_POS);
  }

  private static boolean isElevatorInOperationalRange(double elevatorPosition) {
    return NemesisMathUtil.isBetweenInclusive(
        elevatorPosition,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS,
        Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS);
  }

  private static boolean isArmInOperationalRange(double armPosition) {
    return NemesisMathUtil.isBetweenInclusive(
        armPosition,
        Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS,
        Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS);
  }
}
