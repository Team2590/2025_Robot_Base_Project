# DriveToPose Tuning Guide

This document provides guidance for tuning the DriveToPose functionality to improve alignment accuracy.

## Key Parameters

### PID Constants
```java
// In Constants.java
public class DriveToPoseConstants {
    public static double THETA_kP = 3.0;
    public static double X_kP = 5;
    public static double Y_kP = 5;
    public static double THETA_kI = 0.01;
    public static double X_kI = 0.01;
    public static double Y_kI = 0.01;
}
```

### Vision Parameters
```java
// In VisionConstants.java
public static double linearStdDevBaseline = 0.07;
public static double angularStdDevBaseline = 0.04;
public static double[] cameraStdDevFactorsCloseRange = new double[] {
    0.005, // Upper Source Camera
    0.005, // Processor Camera 
    0.005  // Reef Camera
};
```

## Tuning Recommendations

### 1. PID Tuning
- Start with the default P values
- Gradually increase P until the robot oscillates, then reduce by 20%
- Add small I terms (0.01-0.05) to eliminate steady-state error
- Use D terms sparingly (0.1-0.3) to reduce overshoot

### 2. Vision System Tuning
- Reduce standard deviations for close-range measurements
- Increase trust in cameras with better views of the target
- Filter out ambiguous or invalid poses
- Average multiple measurements when stationary

### 3. Mechanical Checks
- Verify wheel calibration
- Check for wheel slippage
- Ensure center of gravity is balanced
- Confirm all swerve modules are functioning properly

### 4. Control Loop Improvements
- Add integral accumulation for final positioning
- Implement feed-forward to overcome static friction
- Adjust blending between auto and driver control
- Add deadband detection to prevent oscillation

### 5. Testing Methodology
1. Create test patterns with known distances
2. Approach from different angles
3. Log detailed data for each attempt
4. Visualize approach paths and errors
5. Iterate on parameters based on results

## Reference Code

Key implementation details can be found in:
```java:src/main/java/frc/robot/commands/DriveCommands.java
startLine: 423
endLine: 488
```

```java:src/main/java/frc/robot/subsystems/drive/Drive.java
startLine: 88
endLine: 104
```

```java:src/main/java/frc/robot/subsystems/vision/Vision.java
startLine: 100
endLine: 147
```

## Troubleshooting

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Overshooting target | P too high | Reduce P gain |
| Slow to reach target | P too low | Increase P gain |
| Steady-state error | Add I term | Start with 0.01 |
| Oscillation | D too high | Reduce D gain |
| Inconsistent results | Vision noise | Adjust std devs |

## Best Practices
- Test in both simulation and real-world
- Verify parameters work across different field positions
- Monitor system performance during matches
- Keep detailed records of parameter changes
