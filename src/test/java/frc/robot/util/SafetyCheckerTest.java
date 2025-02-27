package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SafetyCheckerTest {

  @Test
  void elevatorMovementSafety_SetPositionsAreSafe_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS));

    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS));

    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS));

    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS));
  }

  @Test
  void armMovementSafety_SetPositionsAreSafe_returnsTrue() {
    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION));

    assertTrue(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS));
  }

  @Test
  void armMovementSafety_OutOfRangeArmPositionsAreUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MIN_POS - 0.1));

    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ARM_MOVEMENT,
            Constants.ArmConstantsLeonidas.ARM_OPERATIONAL_MAX_POS + 0.1));
  }

  @Test
  void elevatorMovementSafety_OutOfRangeElevatorPositionsAreUnsafe_returnsFalse() {
    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MIN_POS - 1));

    assertFalse(
        SafetyChecker.isSafe(
            SafetyChecker.MechanismType.ELEVATOR_MOVEMENT,
            Constants.ElevatorConstantsLeonidas.ELEVATOR_OPERATIONAL_MAX_POS + 1));
  }

  @Test
  void nemesisJoystick_testButtonAlreadyBinded_error() {
    assertThrows(
        IllegalArgumentException.class,
        () -> {
          NemesisJoystick joystick = new NemesisJoystick(1);
          joystick.button(1).onTrue(Commands.none());
          joystick.button(1).onTrue(Commands.none());
        });
  }
}
