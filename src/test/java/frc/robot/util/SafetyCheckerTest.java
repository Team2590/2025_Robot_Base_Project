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
            Constants.ArmConstantsLeonidas
                .ARM_OPERATIONAL_MIN_POS, // armPosition (any value within operational limits)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_DANGER_MAX_POS
                + 1 // Just above danger zone
            ));
  }

  //   @Test
  //   void armMovementSafety_armInSafeRange_returnsTrue() {
  //     assertTrue(
  //         SafetyChecker.isSafe(
  //             SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
  //             Constants.ElevatorConstantsLeonidas.ELEVATOR_DANGER_MAX_POS - 1, // in danger zone
  //             Constants.ArmConstantsLeonidas.ARM_DANGER_MAX_POS + 0.1 // just out of the danger
  // zone
  //             ));
  //   }

  @Test
  void elevatorMovementSafety_AboveOperationalLimit_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS
                + 1, // just above operational limit
            Constants.ArmConstantsLeonidas.ARM_DANGER_MAX_POS + 0.1 // just outside danger zone
            ));
  }

  @Test
  void armMovementSafety_AboveOperationalRange_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS
                + 0.1, // just outside operational limit
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS
                - 1 // just inside operational limit but out of danger zone
            ));
  }

  @Test
  void armMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_DANGER_MIN_POS - 1, // armPosition (below saferange)
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS
                - 1 // below operational min
            ));
  }

  @Test
  void elevatorMovementSafety_armAboveMin_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS,
            Constants.ArmConstantsLeonidas.ARM_DANGER_MAX_POS + 0.1));
  }

  @Test
  void elevatorMovementSafety_elevatorInSafeRange_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_DANGER_MAX_POS
                + 1, // elevatorPosition (within safe range)
            Constants.ArmConstantsLeonidas.ARM_DANGER_MIN_POS - 1 // armPosition (below min)
            ));
  }

  @Test
  void elevatorMovementSafety_bothUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_DANGER_MAX_POS
                - 1, // elevatorPosition (below safe range)
            Constants.ArmConstantsLeonidas.ARM_DANGER_MIN_POS - 1 // armPosition (below min)
            ));
  }

  @Test
  void edgeCase_atMinimumBoundary_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_DANGER_MIN_POS, // Exactly at arm minimum
            Constants.ElevatorConstantsLeonidas
                .ELEVATOR_DANGER_MAX_POS // exactly at edge of danger zone
            ));
  }

  //   @Test
  //   void edgeCase_atMaximumBoundary_returnsTrue() {
  //     assertTrue(
  //         SafetyChecker.isSafe(
  //             SafetyChecker.MechanismType.ARM_MOVEMENT,
  //             Constants.ArmConstantsLeonidas.ARM_DANGER_MAX_POS, // Exactly at arm maximum
  //             Constants.ElevatorConstantsLeonidas
  //                 .ELEVATOR_OPERATIONAL_MAX_POS // Exactly at elevator maximum
  //             ));
  //   }

  //   @Test
  //   void elevatorMovementSafety_moveElevatorDownUnsafeArm_returnsFalse() {
  //     assertFalse(
  //         SafetyChecker.isSafe(
  //             SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
  //             22.0, // slightly unsafe
  //             -0.15 // slightly unsafe
  //             ));
  //   }

  @Test
  void elevatorMovementSafety_moveArmDownUnsafeElevator_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            -0.15, // slightly unsafe
            22.0 // slightly unsafe
            ));
  }
}
