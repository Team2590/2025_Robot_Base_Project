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

      default:
        return false;
    }
  }

  private static boolean isElevatorInOperationalRange(double elevatorPosition) {
    return elevatorPosition >= Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
        && elevatorPosition <= Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS;
  }

  private static boolean isArmInOperationalRange(double armPosition) {
    return armPosition >= Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS
        && armPosition <= Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS;
  }
}
