package frc.robot.util;

import frc.robot.Constants;

public class SafetyChecker {
  public enum MechanismType {
    ARM_ELEVATOR,
    ELEVATOR_ARM,
    // Add other mechanism types as needed
  }

  // Generic safety check for different mechanism types
  public static boolean isSafe(
      MechanismType checkType, double primaryPosition, double secondaryPosition) {
    switch (checkType) {
      case ARM_ELEVATOR:
        return checkArmElevatorSafety(primaryPosition, secondaryPosition);
      case ELEVATOR_ARM:
        return checkElevatorArmSafety(primaryPosition, secondaryPosition);
      default:
        return true; // Default safe if unknown type
    }
  }

  // Specific safety implementations
  private static boolean checkArmElevatorSafety(double elevatorPosition, double armPosition) {
    return elevatorPosition > Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS
        || NemesisMathUtil.isBetweenInclusive(
            armPosition,
            Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS,
            Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS);
  }

  private static boolean checkElevatorArmSafety(double armPosition, double elevatorPosition) {
    return armPosition > Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS
        || NemesisMathUtil.isBetweenInclusive(
            elevatorPosition,
            Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS,
            Constants.ElevatorConstantsLoki.ARM_FACTORY_MAX_POS);
  }

  // Maintain legacy methods for compatibility
  @Deprecated
  public static boolean isArmMovementSafe(double elevatorPosition, double armPosition) {
    return isSafe(MechanismType.ARM_ELEVATOR, elevatorPosition, armPosition);
  }

  @Deprecated
  public static boolean isElevatorMovementSafe(double armPosition, double elevatorPosition) {
    return isSafe(MechanismType.ELEVATOR_ARM, armPosition, elevatorPosition);
  }
}
