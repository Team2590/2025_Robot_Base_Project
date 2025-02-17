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

  public static boolean isSafe(
      MechanismType checkType, double primaryPosition, double secondaryPosition) {
    switch (checkType) {
      case ARM_MOVEMENT:
        // For ARM_MOVEMENT:
        // primaryPosition is elevator position
        // secondaryPosition is arm position
        return isElevatorAboveMin(primaryPosition) || isArmInSafeRange(secondaryPosition);

      case ELEVATOR_MOVEMENT:
        // For ELEVATOR_MOVEMENT:
        // primaryPosition is arm position
        // secondaryPosition is elevator position
        return isArmAboveMin(primaryPosition) || isElevatorInSafeRange(secondaryPosition);

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
}
