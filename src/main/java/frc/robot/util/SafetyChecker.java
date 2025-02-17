package frc.robot.util;

import frc.robot.Constants;

public class SafetyChecker {
  public enum MechanismType {
    ARM_MOVEMENT,
    ELEVATOR_MOVEMENT
  }

  /**
   * Checks if a mechanism movement is safe based on the positions of both the arm and elevator.
   *
   * <p>The safety rules are as follows:
   *
   * <ul>
   *   <li>For ARM_MOVEMENT: Movement is safe if either:
   *       <ul>
   *         <li>The arm is within its operational and safe range (outside 'danger zone'), OR
   *         <li>The elevator is within its safe range.
   *       </ul>
   *   <li>For ELEVATOR_MOVEMENT: Movement is safe if either:
   *       <ul>
   *         <li>The elevator is within its operational and safe range (outside its 'danger zone'),
   *             OR
   *         <li>The arm is within its safe range.
   *       </ul>
   * </ul>
   *
   * @param checkType The type of mechanism movement to check (ARM_MOVEMENT or ELEVATOR_MOVEMENT)
   * @param movingPosition The position of the mechanism that is moving
   * @param otherPosition The position of the other mechanism
   * @return true if the movement is safe, false otherwise
   */
  public static boolean isSafe(
      MechanismType checkType, double movingPosition, double otherPosition) {
    switch (checkType) {
      case ARM_MOVEMENT:
        // For ARM_MOVEMENT:
        // movingPosition is arm position
        // otherPosition is elevator position
        if (!isArmInOperationalRange(movingPosition)) {
          return false; // trying to overshoot the arm position's capabilities
        }

        if (!isArmInSafeRange(movingPosition) && (!isElevatorInSafeRange(otherPosition))) {
          return false; // if target position is out of operational range, or if its in the 'danger
          // zone' and elevator is also in its 'danger zone'.
        }

        return true;

      case ELEVATOR_MOVEMENT:
        // For ELEVATOR_MOVEMENT:
        // movingPosition is elevator position
        // otherPosition is arm position
        if (!isElevatorInOperationalRange(movingPosition)) {
          return false; // outside elevators capabilities
        }

        if (!isElevatorInSafeRange(movingPosition) && !isArmInSafeRange(otherPosition)) {
          return false; // Trying to move elevator to a 'danger zone' position while arm is within
          // its 'danger zone'
        }

        return true;

      default:
        return false;
    }
  }

  private static boolean isElevatorInOperationalRange(double elevatorPosition) {
    return elevatorPosition >= Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
        && elevatorPosition <= Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS;
  }

  private static boolean isElevatorInSafeRange(double elevatorPosition) {
    return elevatorPosition > Constants.ElevatorConstantsLeonidas.ELEVATOR_DANGER_MAX_POS;
  }

  private static boolean isArmInSafeRange(double armPosition) {
    return armPosition > Constants.ArmConstantsLeonidas.ARM_DANGER_MAX_POS;
  }

  private static boolean isArmInOperationalRange(double armPosition) {
    return armPosition >= Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS
        && armPosition <= Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS;
  }
}
