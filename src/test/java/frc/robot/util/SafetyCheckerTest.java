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
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                + 1, // elevatorPosition (above min)
            0.0 // armPosition (any value)
            ));
  }

  @Test
  void armMovementSafety_armInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1, // elevatorPosition (below min)
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS // armPosition (within safe range)
            ));
  }

  @Test
  void armMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1, // elevatorPosition (below min)
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1 // armPosition (below safe range)
            ));
  }

  @Test
  void elevatorMovementSafety_armAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS + 1, // armPosition (above min)
            0.0 // elevatorPosition (any value)
            ));
  }

  @Test
  void elevatorMovementSafety_elevatorInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1, // armPosition (below min)
            Constants.ElevatorConstantsLeonidas
                .ELEVATOR_SAFETY_POS // elevatorPosition (within safe range)
            ));
  }

  @Test
  void elevatorMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS - 1, // armPosition (below min)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1 // elevatorPosition (below safe range)
            ));
  }

  @Test
  void edgeCase_atMinimumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ElevatorConstantsLeonidas
                .ELEVATOR_OPERATIONAL_MIN_POS, // Exactly at elevator minimum
            Constants.ArmConstantsLeonidas.ARM_SAFETY_MIN_POS // Exactly at arm minimum
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
