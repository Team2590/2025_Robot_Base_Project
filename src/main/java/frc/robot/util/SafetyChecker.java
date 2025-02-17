package frc.robot.util;

import frc.robot.Constants;

public class SafetyChecker {
  public enum MechanismType {
    ARM_MOVEMENT,
    ELEVATOR_MOVEMENT
  }

  public static boolean isSafeAlwaysTrue(
      MechanismType checkType, double primaryPosition, double secondaryPosition) {
    return true;
  }

  /**
   * Checks if a mechanism movement is safe based on the positions of both the arm and elevator.
   *
   * <p>The safety rules are as follows:
   *
   * <ul>
   *   <li>For ARM_MOVEMENT: Movement is safe if either:
   *       <ul>
   *         <li>The arm is within its safe range, OR
   *         <li>The elevator is above its minimum operational position
   *       </ul>
   *   <li>For ELEVATOR_MOVEMENT: Movement is safe if either:
   *       <ul>
   *         <li>The elevator is within its safe range, OR
   *         <li>The arm is above its minimum safety position
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
        if (!isArmInSafeRange(movingPosition) && !isElevatorAboveMin(otherPosition)) {
          return false; // Both unsafe
        }
        return isArmInSafeRange(movingPosition) || isElevatorAboveMin(otherPosition);

      case ELEVATOR_MOVEMENT:
        // For ELEVATOR_MOVEMENT:
        // movingPosition is elevator position
        // otherPosition is arm position
        if (!isElevatorInSafeRange(movingPosition) && !isArmAboveMin(otherPosition)) {
          return false; // Both unsafe
        }
        return isElevatorInSafeRange(movingPosition) || isArmAboveMin(otherPosition);

      default:
        return false;
    }
  }

  private static boolean isElevatorAboveMin(double elevatorPosition) {
    return elevatorPosition >= Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS;
  }

  private static boolean isArmInSafeRange(double armPosition) {
    return armPosition >= Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS;
  }

  private static boolean isArmAboveMin(double armPosition) {
    return armPosition >= Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS;
  }

  private static boolean isElevatorInSafeRange(double elevatorPosition) {
    return elevatorPosition >= Constants.ElevatorConstantsLeonidas.ELEVATOR_SAFETY_POS;
  }

  private static boolean isArmBelowSafeRange(double armPosition) {
    return armPosition < Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS;
  }

  private static boolean isElevatorBelowMin(double elevatorPosition) {
    return elevatorPosition < Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS;
  }

  private static boolean isElevatorBelowSafeRange(double elevatorPosition) {
    return elevatorPosition < Constants.ElevatorConstantsLeonidas.ELEVATOR_SAFETY_POS;
  }

  private static boolean isArmBelowMin(double armPosition) {
    return armPosition < Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS;
  }
}
