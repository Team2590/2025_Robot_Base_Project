package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;

public class ArmPositionManager {
  private static final double TICKS_PER_ROTATION = 4096.0; // Standard for CTRE CANcoder
  private final double
      gearRatio; // Sensor rotations per arm rotation (e.g., 1.2 means it takes 1.2 sensor rotations
  // for 1 arm rotation)
  private final int maxRotations; // Maximum allowed full rotations in either direction
  private final double
      magnetOffset; // Offset of the CANcoder's absolute position when arm is at zero degrees
  private int rotationCount = 0; // Tracks total number of full rotations
  private double lastRawPosition = 0; // Last raw position from CANcoder (0-1)
  private boolean isFirstUpdate = true; // Flag for initial position update
  private double prevNormalizedPosition = 0; // Stores position before normalization

  // Boundary detection thresholds
  private static final double BOUNDARY_CROSSING_THRESHOLD =
      0.5; // Minimum position change to detect boundary crossing
  private static final double NEAR_ONE_THRESHOLD =
      0.01; // Range around 1.0 to treat as 0.0 (0.99 to 1.01)
  private static final double ZERO_TOLERANCE = 0.001; // Tolerance for comparing to 0 degrees

  /**
   * Creates a new ArmPositionManager
   *
   * @param sensorToMechanismRatio The ratio of sensor rotations to arm rotations (e.g., 1.2 means
   *     1.2 sensor rotations per arm rotation)
   * @param maxRotations Maximum number of full arm rotations allowed in either direction
   * @param magnetOffset The CANcoder's absolute position when arm is at zero degrees (0-1)
   */
  public ArmPositionManager(double sensorToMechanismRatio, int maxRotations, double magnetOffset) {
    this.gearRatio = sensorToMechanismRatio;
    this.maxRotations = maxRotations;
    this.magnetOffset = magnetOffset;
    // Explicitly ensure we start with 0 values
    this.lastRawPosition = magnetOffset; // Start at zero position
    this.rotationCount = 0;
  }

  /**
   * Updates the rotation tracking based on the current Cancoder position
   *
   * @param currentRawPosition Current raw position from Cancoder (0-1)
   * @return Current absolute angle in degrees (can be > 360 or < 0 based on rotations)
   */
  public double updateRotationTracking(double currentRawPosition) {
    // Keep track of the raw position before normalization
    prevNormalizedPosition = currentRawPosition;

    // Handle the special case of exactly 1.0 to avoid extra rotation detection
    if (currentRawPosition >= 0.99 && currentRawPosition <= 1.01) {
      currentRawPosition = 0.0;
    } else {
      currentRawPosition = MathUtil.inputModulus(currentRawPosition, 0, 1);
    }

    // Skip boundary crossing logic on first update
    if (isFirstUpdate) {
      isFirstUpdate = false;
      prevNormalizedPosition = currentRawPosition;
      lastRawPosition = currentRawPosition;
      return getAbsoluteAngleDegrees();
    }

    // Only detect boundary crossings if values are significantly different
    // to avoid tiny floating point variations causing false crossings
    if (Math.abs(currentRawPosition - lastRawPosition) > BOUNDARY_CROSSING_THRESHOLD) {
      // If we went from high to low (e.g., 0.9 to 0.1), we're going clockwise
      if (lastRawPosition > currentRawPosition) {
        rotationCount++;
      } else {
        // If we went from low to high (e.g., 0.1 to 0.9), we're going counter-clockwise
        rotationCount--;
      }
      System.out.println("DEBUG: Rotation count updated to " + rotationCount);
    }
    lastRawPosition = currentRawPosition;
    prevNormalizedPosition = currentRawPosition;
    return getAbsoluteAngleDegrees();
  }

  /**
   * Gets the current absolute angle including all rotations
   *
   * @return Current absolute angle in degrees
   */
  public double getAbsoluteAngleDegrees() {
    // First, get the total sensor rotations including full rotations and current position
    double totalSensorRotations = rotationCount + lastRawPosition;

    // Adjust for magnet offset
    double offsetAdjustedRotations = totalSensorRotations - magnetOffset;

    // Convert sensor rotations to arm rotations using gear ratio
    double armRotations = offsetAdjustedRotations / gearRatio;

    // Convert to degrees
    double degrees = armRotations * 360.0;

    System.out.println(
        "DEBUG: getAbsoluteAngleDegrees - rawPos: "
            + lastRawPosition
            + ", totalSensorRot: "
            + totalSensorRotations
            + ", armRot: "
            + armRotations
            + ", rotations: "
            + rotationCount
            + ", result: "
            + degrees);

    return degrees;
  }

  /**
   * Gets the normalized angle between 0-360 degrees
   *
   * @return Normalized angle in degrees
   */
  public double getNormalizedAngleDegrees() {
    double angle = getAbsoluteAngleDegrees();
    double normalized = MathUtil.inputModulus(angle, 0, 360);
    // Special case for 0 and 360 - always return 0 for initial state
    if (Math.abs(normalized) < ZERO_TOLERANCE || Math.abs(normalized - 360.0) < ZERO_TOLERANCE) {
      return 0.0;
    }
    return normalized;
  }

  /**
   * Converts a target angle in degrees to Cancoder position units
   *
   * @param targetAngleDegrees Desired angle in degrees
   * @return Cancoder position (0-1) accounting for gear ratio and magnet offset
   */
  public double degreesToCancoderPosition(double targetAngleDegrees) {
    // Convert target degrees to arm rotations
    double targetArmRotations = targetAngleDegrees / 360.0;

    // Convert arm rotations to sensor rotations using gear ratio
    double targetSensorRotations = targetArmRotations * gearRatio;

    // Add magnet offset and normalize to 0-1
    return MathUtil.inputModulus(targetSensorRotations + magnetOffset, 0, 1);
  }

  /**
   * Checks if a move to the target angle is safe given rotation limits
   *
   * @param targetAngleDegrees Desired angle in degrees
   * @return true if the move is safe, false otherwise
   */
  public boolean isSafeToMove(double targetAngleDegrees) {
    // For safety calculation, we use the absolute value of the angle
    // This handles both positive and negative rotation directions
    double absoluteAngle = Math.abs(targetAngleDegrees);

    // Calculate how many rotations this represents (possibly fractional)
    double rotations = absoluteAngle / 360.0;

    // The move is safe if the number of rotations is less than or equal to maxRotations
    return rotations <= maxRotations;
  }

  /**
   * Calculates the shortest path to reach a target angle
   *
   * @param targetAngleDegrees Desired angle in degrees (0-360)
   * @return The target angle modified to take the shortest path
   */
  public double calculateShortestPath(double targetAngleDegrees) {
    double normalizedTarget = MathUtil.inputModulus(targetAngleDegrees, 0, 360);
    double currentNormalized = getNormalizedAngleDegrees();

    // Calculate the difference between current and target angles
    double diff = normalizedTarget - currentNormalized;

    // Adjust the difference to take the shortest path
    if (diff > 180) {
      diff -= 360;
    } else if (diff < -180) {
      diff += 360;
    }

    // Get the current absolute angle and its whole rotations
    double currentAngle = getAbsoluteAngleDegrees();
    double currentRotations = Math.floor(currentAngle / 360.0);
    double currentBase = currentRotations * 360.0;

    // Calculate the target angle that maintains the current rotation count
    double targetAngle = currentBase + normalizedTarget;

    // If the normalized diff is negative and large, we might need to go to the next rotation
    if (diff < -150 && normalizedTarget > 270) {
      targetAngle = (currentRotations - 1) * 360.0 + normalizedTarget;
    }
    // If the normalized diff is positive and large, we might need to go back a rotation
    else if (diff > 150 && normalizedTarget < 90) {
      targetAngle = (currentRotations + 1) * 360.0 + normalizedTarget;
    }

    return targetAngle;
  }

  /**
   * Calculates both possible paths (clockwise and counter-clockwise) to reach a target angle and
   * returns the safe path if one exists
   *
   * @param targetAngleDegrees Desired angle in degrees (0-360)
   * @return The target angle modified to take a safe path, or -1 if no safe path exists
   */
  public double calculateSafePath(double targetAngleDegrees) {
    // Normalize the target angle to 0-360 range
    double normalizedTarget = MathUtil.inputModulus(targetAngleDegrees, 0, 360);
    double currentNormalized = getNormalizedAngleDegrees();
    double currentAngle = getAbsoluteAngleDegrees();

    // Calculate both possible paths: clockwise and counter-clockwise

    // For clockwise movement:
    double clockwiseDiff = normalizedTarget - currentNormalized;
    if (clockwiseDiff <= 0) {
      clockwiseDiff += 360;
    }
    double clockwisePath = currentAngle + clockwiseDiff;

    // For counter-clockwise movement:
    double counterClockwiseDiff = normalizedTarget - currentNormalized;
    if (counterClockwiseDiff >= 0) {
      counterClockwiseDiff -= 360;
    }
    double counterClockwisePath = currentAngle + counterClockwiseDiff;

    // Check if either path exceeds rotation limits
    boolean clockwiseSafe = isSafeToMove(clockwisePath);
    boolean counterClockwiseSafe = isSafeToMove(counterClockwisePath);

    System.out.println(
        "DEBUG: Safe Path - Current: "
            + currentAngle
            + ", Target: "
            + normalizedTarget
            + ", Clockwise: "
            + clockwisePath
            + " (safe: "
            + clockwiseSafe
            + ")"
            + ", Counter: "
            + counterClockwisePath
            + " (safe: "
            + counterClockwiseSafe
            + ")");

    // If both paths are safe, return the shorter one
    if (clockwiseSafe && counterClockwiseSafe) {
      return Math.abs(clockwiseDiff) <= Math.abs(counterClockwiseDiff)
          ? clockwisePath
          : counterClockwisePath;
    }
    // If only one path is safe, return that one
    else if (clockwiseSafe) {
      return clockwisePath;
    } else if (counterClockwiseSafe) {
      return counterClockwisePath;
    }

    // No safe path exists
    System.out.println("DEBUG: No safe path exists to target " + normalizedTarget);
    return -1;
  }

  /**
   * Gets the current rotation count
   *
   * @return Number of full rotations (positive or negative)
   */
  public int getRotationCount() {
    return rotationCount;
  }

  /**
   * Resets the rotation tracking to a specific angle
   *
   * @param angleDegrees The angle to reset to (in degrees)
   */
  public void resetToAngle(double angleDegrees) {
    // Convert target degrees to arm rotations
    double targetArmRotations = angleDegrees / 360.0;

    // Convert arm rotations to sensor rotations using gear ratio
    double targetSensorRotations = targetArmRotations * gearRatio;

    // Set rotation count to the whole number of rotations
    rotationCount = (int) Math.floor(targetSensorRotations);

    // Calculate the fractional part and add the magnet offset
    double fractionalRotation = targetSensorRotations - rotationCount;
    lastRawPosition = MathUtil.inputModulus(fractionalRotation + magnetOffset, 0, 1);

    prevNormalizedPosition = lastRawPosition;
    isFirstUpdate = true;
  }
}
