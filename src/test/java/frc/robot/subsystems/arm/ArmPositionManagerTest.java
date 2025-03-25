package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmPositionManagerTest {
  private ArmPositionManager armManager;
  private static final double DELTA = 0.001; // Tolerance for floating-point comparisons
  private static final double GEAR_RATIO = 1.2; // 1.2 sensor rotations per arm rotation
  private static final int MAX_ROTATIONS = 2; // Maximum allowed full rotations in either direction
  private static final double MAGNET_OFFSET =
      0.25; // Example offset where 0.25 corresponds to 0 degrees

  // Helper methods to calculate expected values
  private double calculateExpectedDegrees(double sensorPosition) {
    // Calculate degrees from sensor position considering gear ratio and offset
    double offsetAdjustedRotations = sensorPosition - MAGNET_OFFSET;
    return (offsetAdjustedRotations / GEAR_RATIO) * 360.0;
  }

  private double calculateSensorPosition(double degrees) {
    // Calculate sensor position from degrees considering gear ratio and offset
    double armRotations = degrees / 360.0;
    double sensorRotations = armRotations * GEAR_RATIO;
    return MathUtil.inputModulus(sensorRotations + MAGNET_OFFSET, 0, 1);
  }

  private double normalizeAngle(double degrees) {
    return MathUtil.inputModulus(degrees, 0, 360);
  }

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
    // When CANcoder reads the offset value, we should get 0 degrees
    armManager.updateRotationTracking(MAGNET_OFFSET);
    assertEquals(0.0, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(0.0, armManager.getNormalizedAngleDegrees(), DELTA);

    // When CANcoder reads 0.0, calculate expected degrees
    double expectedDegrees = calculateExpectedDegrees(0.0);
    double expectedNormalized = normalizeAngle(expectedDegrees);

    armManager.updateRotationTracking(0.0);
    assertEquals(expectedDegrees, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(expectedNormalized, armManager.getNormalizedAngleDegrees(), DELTA);

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

    // Calculate expected degrees at 0.85
    expectedDegrees = calculateExpectedDegrees(0.85);
    assertEquals(expectedDegrees, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(normalizeAngle(expectedDegrees), armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testGearRatio() {
    // For 90 degrees of arm rotation, calculate expected sensor position
    double expectedPos90 = calculateSensorPosition(90.0);
    assertEquals(expectedPos90, armManager.degreesToCancoderPosition(90.0), DELTA);

    // For 360 degrees (full rotation), calculate expected sensor position
    double expectedPos360 = calculateSensorPosition(360.0);
    assertEquals(expectedPos360, armManager.degreesToCancoderPosition(360.0), DELTA);
  }

  @Test
  void testUpdateRotationTrackingClockwise() {
    // Start at zero position (magnet offset)
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Move clockwise past boundary
    armManager.updateRotationTracking(0.85);
    armManager.updateRotationTracking(0.05);

    // Calculate expected rotation count based on total angle traveled
    // With GEAR_RATIO of 1.2, we need 1.2 sensor rotations to make one full arm rotation
    // So moving from 0.25 to 0.85 and then to 0.05 (crossing boundary) is less than a full rotation
    int expectedRotations = 0;

    // Calculate expected degrees and normalize
    double expectedDegrees = calculateExpectedDegrees(0.05 + expectedRotations);
    double expectedNormalized = normalizeAngle(expectedDegrees);

    assertEquals(expectedRotations, armManager.getRotationCount());
    assertEquals(expectedNormalized, armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testUpdateRotationTrackingCounterClockwise() {
    // Start at zero position (magnet offset)
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Move counter-clockwise past boundary
    armManager.updateRotationTracking(0.05);
    armManager.updateRotationTracking(0.85);

    // With counter-clockwise motion past boundary, we should get rotation count of -1
    int expectedRotations = -1;

    // Calculate expected degrees and normalize
    double expectedDegrees = calculateExpectedDegrees(0.85 + expectedRotations);
    double expectedNormalized = normalizeAngle(expectedDegrees);

    assertEquals(expectedRotations, armManager.getRotationCount());
    assertEquals(expectedNormalized, armManager.getNormalizedAngleDegrees(), DELTA);
  }

  @Test
  void testSafetyLimits() {
    // Within limits
    assertTrue(armManager.isSafeToMove(360.0));
    assertTrue(armManager.isSafeToMove(-360.0));
    assertTrue(armManager.isSafeToMove(MAX_ROTATIONS * 360.0)); // Max allowed rotations
    assertTrue(armManager.isSafeToMove(-MAX_ROTATIONS * 360.0)); // Negative max allowed rotations

    // Beyond limits
    assertFalse(armManager.isSafeToMove((MAX_ROTATIONS + 1) * 360.0)); // Beyond max
    assertFalse(armManager.isSafeToMove(-(MAX_ROTATIONS + 1) * 360.0)); // Beyond negative max
  }

  @Test
  void testShortestPath() {
    // Start at zero position
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Test shortest path to 90 degrees
    assertEquals(90.0, armManager.calculateShortestPath(90.0), DELTA);

    // Test shortest path to 270 degrees
    // The shortest path determination should work regardless of gear ratio
    double expectedPath270 = 270.0;
    assertEquals(expectedPath270, armManager.calculateShortestPath(270.0), DELTA);

    // Test path calculation after a full rotation
    double fullRotationPlusAngle = 400.0;
    armManager.resetToAngle(fullRotationPlusAngle);

    // When at 400 degrees, shortest path to 90 should be 450 (maintain rotation count)
    double expectedPathAfterRotation = 450.0;
    assertEquals(expectedPathAfterRotation, armManager.calculateShortestPath(90.0), DELTA);
  }

  @Test
  void testMultipleRotations() {
    // Start at zero position
    armManager.updateRotationTracking(MAGNET_OFFSET);

    // Calculate how many passes we need to complete 2 full rotations
    // Each boundary crossing increments the rotation count
    int passesNeeded = (int) Math.ceil(2.0 * GEAR_RATIO);

    // With our gear ratio, we need multiple passes to ensure 2 full rotations
    for (int i = 0; i < passesNeeded; i++) {
      // Complete a clockwise pass
      for (double pos = MAGNET_OFFSET; pos < 0.95; pos += 0.1) {
        armManager.updateRotationTracking(pos);
      }
      armManager.updateRotationTracking(0.95);
      armManager.updateRotationTracking(0.05);
    }

    // After enough passes, we should have at least 2 rotations
    assertTrue(armManager.getRotationCount() >= 2);

    // The absolute angle should be at least 720 degrees (2 full rotations)
    double minExpectedDegrees = 720.0;
    assertTrue(armManager.getAbsoluteAngleDegrees() >= minExpectedDegrees);
  }

  @Test
  void testResetToAngle() {
    // Create test angle with multiple rotations plus offset
    double testAngle = 725.0;
    armManager.resetToAngle(testAngle);

    // Verify absolute and normalized angles
    assertEquals(testAngle, armManager.getAbsoluteAngleDegrees(), DELTA);
    assertEquals(normalizeAngle(testAngle), armManager.getNormalizedAngleDegrees(), DELTA);

    // Calculate expected sensor position
    double armRotations = testAngle / 360.0;
    double expectedRotCount = (int) Math.floor(armRotations * GEAR_RATIO);
    double fractionalRotation = (armRotations * GEAR_RATIO) - expectedRotCount;
    double expectedPosition = MathUtil.inputModulus(fractionalRotation + MAGNET_OFFSET, 0, 1);

    // Verify calculated position matches our helper method
    assertEquals(
        calculateSensorPosition(testAngle), armManager.degreesToCancoderPosition(testAngle), DELTA);
  }

  @Test
  void testNonIntegerGearRatio() {
    // Create a new manager with a different gear ratio
    double testGearRatio = 1.33333;
    ArmPositionManager complexManager =
        new ArmPositionManager(testGearRatio, MAX_ROTATIONS, MAGNET_OFFSET);

    // Calculate expected sensor position for 90 degrees with this gear ratio
    double armRotations = 90.0 / 360.0;
    double sensorRotations = armRotations * testGearRatio;
    double expectedPos = MathUtil.inputModulus(sensorRotations + MAGNET_OFFSET, 0, 1);

    // Verify position calculation
    double pos = complexManager.degreesToCancoderPosition(90.0);
    assertEquals(expectedPos, pos, DELTA);

    // Verify angle calculation
    complexManager.updateRotationTracking(pos);
    assertEquals(90.0, complexManager.getAbsoluteAngleDegrees(), DELTA);
  }

  @Test
  void testSafePathWhenShortestIsAlsoSafest() {
    // Set up initial position at 90 degrees
    armManager.resetToAngle(90.0);

    // Try to move to 180 degrees
    // The shortest path is clockwise 90° which is well within rotation limits
    // The long path would be counter-clockwise -270° which is also within limits
    double safePath = armManager.calculateSafePath(180.0);

    // Should choose the shortest (clockwise) path since both are safe
    assertEquals(180.0, safePath, DELTA);
  }

  @Test
  void testSafePathWhenShortestIsUnsafe() {
    // Set up a position near max rotation limit
    double startAngle = (MAX_ROTATIONS * 360.0) - 45.0; // 675 degrees
    armManager.resetToAngle(startAngle);

    // Try to move to a position that would exceed max rotation if taken clockwise
    // If we go clockwise (shortest path) by +90°, we'd be at 765°
    // If we go counter-clockwise (long way) by -270°, we'd be at 405°
    double safePath = armManager.calculateSafePath(normalizeAngle(startAngle + 90.0));

    // Should choose the counter-clockwise path as it's within safe limit
    assertTrue(armManager.isSafeToMove(safePath));
    assertTrue(safePath < 500.0); // Should be around 405°
    assertTrue(safePath > 400.0); // Should be around 405°
  }

  @Test
  void testSafePathWhenNoSafePathExists() {
    // Create a very restrictive manager with max 0 rotations
    // This means angles above 0° are unsafe
    ArmPositionManager restrictedManager = new ArmPositionManager(GEAR_RATIO, 0, MAGNET_OFFSET);

    // Try to move to any non-zero angle
    double noSafePath = restrictedManager.calculateSafePath(90.0);

    // Should return -1 as no path is safe (any path > 0° exceeds our 0 rotation limit)
    assertEquals(-1, noSafePath, DELTA);
  }
}
