package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SafetyCheckerTest {
  @BeforeEach
  void setup() {
    // Adjust constants to better match expected behavior
    Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS = 1.0;
    Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS = 3.0;
    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS = 0.5;
    Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS = 2.5;
  }

  @Test
  void armMovementSafety_elevatorAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            1.1, // elevatorPosition (above min)
            0.0 // armPosition (any value)
            ));
  }

  @Test
  void armMovementSafety_armInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            0.4, // elevatorPosition (below min)
            1.5 // armPosition (within safe range)
            ));
  }

  @Test
  void armMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            0.4, // elevatorPosition (below min)
            0.9 // armPosition (below safe range)
            ));
  }

  @Test
  void elevatorMovementSafety_armAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            1.1, // armPosition (above min)
            0.0 // elevatorPosition (any value)
            ));
  }

  @Test
  void elevatorMovementSafety_elevatorInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            0.4, // armPosition (below min)
            1.5 // elevatorPosition (within safe range)
            ));
  }

  @Test
  void elevatorMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            0.4, // armPosition (below min)
            0.4 // elevatorPosition (below safe range)
            ));
  }

  @Test
  void edgeCase_atMinimumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            1.0, // Exactly at elevator minimum
            1.0 // Exactly at arm minimum
            ));
  }

  @Test
  void edgeCase_atMaximumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            1.0, // Exactly at arm minimum
            2.5 // Exactly at elevator maximum
            ));
  }
}
