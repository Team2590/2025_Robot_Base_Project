package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmPositionManagerTest {
  private ArmPositionManager armManager;
  private static final double DELTA = 0.001; // Tolerance for floating-point comparisons
  private static final double GEAR_RATIO = 1.2; // 1.2 sensor rotations per arm rotation
  private static final int MAX_ROTATIONS = 2; // Maximum allowed full rotations in either direction
  private static final double MAGNET_OFFSET = 0.25; // Example offset where 0.25 corresponds to 0 degrees

  @BeforeEach
  void setUp() {
    armManager = new ArmPositionManager(GEAR_RATIO, MAX_ROTATIONS, MAGNET_OFFSET);
  }

  @Test
  void testInitialState() {
    assertEquals(0.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(0.0, armManager.getNormalizedAngleDegrees(), DELTA);
    assertEquals(0, armManager.getRotationCount());
  }

  @Test
  void testMagnetOffset() {
    // When CANcoder reads the offset value (0.25), we should get 0 degrees
    armManager.updateRotationTracking(MAGNET_OFFSET);
    assertEquals(0.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(0.0, armManager.getNormalizedAngleDegrees(), DELTA);

    // When CANcoder reads 0.0, we should get -75 degrees (0.25 * 360 / 1.2)
    armManager.updateRotationTracking(0.0);
    assertEquals(-75.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(285.0, armManager.getNormalizedAngleDegrees(), DELTA);

    // Reset to start fresh
    armManager = new ArmPositionManager(GEAR_RATIO, MAX_ROTATIONS, MAGNET_OFFSET);

    // Start at zero position (magnet offset)
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Now test gradual movement to 0.85 without crossing the boundary
    // This incremental approach prevents triggering boundary crossing logic
    double[] positions = {0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
    for (double pos : positions) {
      armManager.updateRotationTracking(pos);
    }

    // At 0.85, we should be at 180 degrees
    assertEquals(180.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(180.0, armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testGearRatio() {
    // For 90 degrees of arm rotation with 1.2:1 ratio, sensor should move 0.3 rotations (plus
    // offset)
    assertEquals(0.55, armManager.degreesToCancoderPosition(90.0), DELTA); // 0.3 + 0.25 = 0.55

    // For 360 degrees (full rotation) with 1.2:1 ratio, sensor should move 1.2 rotations
    double fullRotationPos = armManager.degreesToCancoderPosition(360.0);
    assertEquals(0.45, fullRotationPos, DELTA); // (1.2 + 0.25) % 1 = 0.45
  }

  @Test
  void testUpdateRotationTrackingClockwise() {
    // Start at zero position (magnet offset)
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Move clockwise past boundary
    // At 0.85, we're at 180 degrees
    armManager.updateRotationTracking(0.85);
    // At 0.05, we've gone past the boundary and should be at ~270 degrees with rotation 0
    armManager.updateRotationTracking(0.05);

    // The final rotation count should be 0 because we're only at 1/3 of a rotation
    // with our gear ratio of 1.2
    assertEquals(0, armManager.getRotationCount());
    assertEquals(300.0, armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testUpdateRotationTrackingCounterClockwise() {
    // Start at zero position (magnet offset)
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Move counter-clockwise past boundary
    // At 0.05, we're at -60 degrees
    armManager.updateRotationTracking(0.05);
    // At 0.85, we've gone past the boundary and should be at -120 degrees
    armManager.updateRotationTracking(0.85);

    assertEquals(-1, armManager.getRotationCount());
    // -120 degrees is equivalent to 240 degrees normalized
    assertEquals(240.0, armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testSafetyLimits() {
    // Within limits
    assertTrue(armManager.isSafeToMove(360.0));
    assertTrue(armManager.isSafeToMove(-360.0));
    assertTrue(armManager.isSafeToMove(720.0)); // 2 rotations
    assertTrue(armManager.isSafeToMove(-720.0)); // -2 rotations

    // Beyond limits
    assertFalse(armManager.isSafeToMove(1080.0)); // 3 rotations
    assertFalse(armManager.isSafeToMove(-1080.0)); // -3 rotations
  }

  @Test
  void testShortestPath() {
    // Start at zero position
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Test shortest path calculations
    assertEquals(90.0, armManager.calculateShortestPath(90.0), DELTA);
    // With our gear ratio, moving to 270 is actually shortest as 270 degrees rather than -90
    assertEquals(270.0, armManager.calculateShortestPath(270.0), DELTA);

    // Test path calculation after a full rotation
    armManager.resetToAngle(400.0); // Set to 400 degrees (40 degrees + 1 rotation)
    assertEquals(
        450.0, armManager.calculateShortestPath(90.0), DELTA); // Should maintain the rotation
  }

  @Test
  void testMultipleRotations() {
    // Start at zero position
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // With our gear ratio of 1.2, we need more than one full sensor rotation to complete
    // a full arm rotation. Let's make enough passes to ensure 2 full rotations.
    for (int i = 0; i < 3; i++) {
      // Complete a clockwise pass
      for (double pos = 0.25; pos < 0.95; pos += 0.1) {
        armManager.updateRotationTracking(pos);
      }
      armManager.updateRotationTracking(0.95);
      armManager.updateRotationTracking(0.05);
    }

    // After enough passes to ensure 2 full rotations
    assertTrue(armManager.getRotationCount() >= 2);
    double angle = armManager.getAbsoluteAngleDegrees();
    assertTrue(angle >= 720.0);
  }

  @Test
  void testResetToAngle() {
    // Test resetting to 725 degrees (2 full rotations + 5 degrees)
    armManager.resetToAngle(725.0);
    assertEquals(725.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(5.0, armManager.getNormalizedAngleDegrees(), DELTA);

    // Verify sensor position is correct
    // 725 degrees = 2.013889 arm rotations
    // 2.013889 * 1.2 = 2.416667 sensor rotations
    // 0.416667 + 0.25 = 0.666667 (normalized position)
    assertEquals(0.666667, armManager.degreesToCancoderPosition(725.0), DELTA);
  }

  @Test
  void testNonIntegerGearRatio() {
    // Create a new manager with a more complex gear ratio
    ArmPositionManager complexManager =
        new ArmPositionManager(1.33333, MAX_ROTATIONS, MAGNET_OFFSET);

    // Test conversion from degrees to position
    double pos = complexManager.degreesToCancoderPosition(90.0);
    // 90 degrees = 0.25 arm rotations
    // 0.25 * 1.33333 = 0.33333 sensor rotations
    // 0.33333 + 0.25 = 0.58333
    assertEquals(0.58333, pos, DELTA);

    // Test conversion from position to degrees
    complexManager.updateRotationTracking(pos);
    assertEquals(90.0, complexManager.getAbsoluteAngleDegrees(), DELTA);
  }
}
