package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SafetyCheckerTest {

  @Test
  void armMovementSafety_elevatorAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            1.0, // armPosition (any value)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_SAFETY_POS
                + 1 // elevatorPosition (above min)
            ));
  }

  @Test
  void armMovementSafety_armInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS
                + 1, // armPosition (within safe range)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1 // elevatorPosition (below min)
            ));
  }

  @Test
  void armMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1, // armPosition (below safe range)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1 // elevatorPosition (below min)
            ));
  }

  @Test
  void elevatorMovementSafety_armAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            1.0, // elevatorPosition (any value)
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS + 1 // armPosition (above min)
            ));
  }

  @Test
  void elevatorMovementSafety_elevatorInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_SAFETY_POS
                + 1, // elevatorPosition (within safe range)
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1 // armPosition (below min)
            ));
  }

  @Test
  void elevatorMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_SAFETY_POS
                - 1, // elevatorPosition (below safe range)
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1 // armPosition (below min)
            ));
  }

  @Test
  void edgeCase_atMinimumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS, // Exactly at arm minimum
            Constants.ElevatorConstantsLeonidas
                .ELEVATOR_OPERATIONAL_MIN_POS // Exactly at elevator minimum
            ));
  }

  @Test
  void edgeCase_atMaximumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS, // Exactly at arm minimum
            Constants.ElevatorConstantsLeonidas
                .ELEVATOR_OPERATIONAL_MAX_POS // Exactly at elevator maximum
            ));
  }
}
