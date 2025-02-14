package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SafetyCheckerTest {
  @BeforeEach
  void setup() {
    // Reset constants to default test values before each test
    Constants.ElevatorConstantsLoki.ARM_FACTORY_MIN_POS = 1.0;
    Constants.ElevatorConstantsLoki.ARM_FACTORY_MAX_POS = 3.0;
    Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MIN_POS = 0.5;
    Constants.ArmConstantsLoki.ELEVATOR_FACTORY_MAX_POS = 2.5;
  }

  @Test
  void armElevatorSafety_elevatorAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_ELEVATOR,
            1.1, // elevatorPosition (above min)
            0.0 // armPosition (any value)
            ));
  }

  @Test
  void armElevatorSafety_armInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_ELEVATOR,
            0.9, // elevatorPosition (below min)
            1.0 // armPosition (within arm safe range)
            ));
  }

  @Test
  void armElevatorSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_ELEVATOR,
            0.9, // elevatorPosition (below min)
            0.4 // armPosition (below arm safe range)
            ));
  }

  @Test
  void elevatorArmSafety_armAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_ARM,
            0.6, // armPosition (above min)
            0.0 // elevatorPosition (any value)
            ));
  }

  @Test
  void elevatorArmSafety_elevatorInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_ARM,
            0.4, // armPosition (below min)
            1.5 // elevatorPosition (within elevator safe range)
            ));
  }

  @Test
  void elevatorArmSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_ARM,
            0.4, // armPosition (below min)
            0.4 // elevatorPosition (below elevator safe range)
            ));
  }

  @Test
  void edgeCase_atMinimumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_ELEVATOR,
            1.0, // Exactly at elevator minimum
            0.5 // Exactly at arm minimum
            ));
  }

  @Test
  void edgeCase_atMaximumBoundary_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_ARM,
            0.5, // Exactly at arm minimum
            3.0 // Exactly at elevator maximum
            ));
  }
}
