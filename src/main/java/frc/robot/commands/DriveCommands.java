// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveToPoseConstraints;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Alignment tolerances
  private static final double X_ALIGNMENT_TOLERANCE = 0.02; // meters
  private static final double Y_ALIGNMENT_TOLERANCE = 0.02; // meters
  private static final double ROTATION_ALIGNMENT_TOLERANCE = 0.05; // radians

  private DriveCommands() {}

  /**
   * Logger dedicated to determining if the robot is aligned to the target pose. Returns true if all
   * alignment errors are within tolerance, false otherwise
   */
  private static void logAlignmentData(Pose2d currentPose, Pose2d targetPose, String phase) {
    // Calculate errors
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError =
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Check if within tolerances
    boolean xAligned = Math.abs(xError) <= X_ALIGNMENT_TOLERANCE;
    boolean yAligned = Math.abs(yError) <= Y_ALIGNMENT_TOLERANCE;
    boolean rotationAligned = Math.abs(rotationError) <= ROTATION_ALIGNMENT_TOLERANCE;
    boolean fullyAligned = xAligned && yAligned && rotationAligned;

    // Log all data to Shuffleboard
    Logger.recordOutput("Alignment/Phase", phase);
    Logger.recordOutput("Alignment/CurrentPose", currentPose);
    Logger.recordOutput("Alignment/TargetPose", targetPose);
    Logger.recordOutput("Alignment/XError", xError);
    Logger.recordOutput("Alignment/YError", yError);
    Logger.recordOutput("Alignment/RotationError", rotationError);
    Logger.recordOutput("Alignment/XAligned", xAligned);
    Logger.recordOutput("Alignment/YAligned", yAligned);
    Logger.recordOutput("Alignment/RotationAligned", rotationAligned);
    Logger.recordOutput("Alignment/FullyAligned", fullyAligned);
    Logger.recordOutput("Alignment/XErrorPercent", Math.abs(xError) / X_ALIGNMENT_TOLERANCE);
    Logger.recordOutput("Alignment/YErrorPercent", Math.abs(yError) / Y_ALIGNMENT_TOLERANCE);
    Logger.recordOutput(
        "Alignment/RotationErrorPercent", Math.abs(rotationError) / ROTATION_ALIGNMENT_TOLERANCE);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveSlow(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double scaleFactor) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec() * scaleFactor,
                  ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec() * scaleFactor,
                  omegaSupplier.getAsDouble() * drive.getMaxAngularSpeedRadPerSec() * scaleFactor);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * RobotContainer.constantsWrapper.driveBaseRadius)
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Command driveToPose(Pose2d targetPose) {
    System.out.println("DRIVING TO POSE " + targetPose);
    return AutoBuilder.pathfindToPose(targetPose, DriveToPoseConstraints.fastpathConstraints, 0.0);
  }

  public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    System.out.println("DRIVING TO POSE " + targetPoseSupplier.get());
    HashSet<Subsystem> requirements = new HashSet<>();
    requirements.add(drive);
    return Commands.defer(
        () -> {
          Pose2d targetPose =
              targetPoseSupplier
                  .get()
                  .plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));
          if (targetPose != null) {
            Logger.recordOutput("DriveCommands/drive_to_pose_target", targetPose);
            return AutoBuilder.pathfindToPose(
                targetPose, DriveToPoseConstraints.fastpathConstraints, 0.0);
          }
          return Commands.print("No target pose found, not running the command");
        },
        requirements);
  }

  /**
   * Positions the robot at a fixed distance from the target pose along a line defined by the target
   * pose's rotation. Once that position is reached, the robot will proceed to the actual target
   * pose. If targetDistance is 0, robot will go directly to the target pose.
   *
   * @param drive robot drive subsystem
   * @param forwardSupplier supplier for forward/backward movement (typically joystick Y-axis)
   * @param strafeSupplier supplier for left/right movement (typically joystick X-axis)
   * @param targetPoseSupplier supplier for the target pose
   * @param targetDistance distance in meters to stop from the target (0 to go directly to target)
   * @return command that drives to the calculated position
   */
  public static Command alignToTargetLine(
      Drive drive,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      double targetDistance) {

    // Create PID controllers for position control

    // State tracking - make it a final array so we can modify it inside the lambda
    final boolean[] reachedInitialPosition = {false};
    final double positionThreshold = 0.01; // meters

    // If targetDistance is 0, skip the first phase
    if (targetDistance == 0) {
      reachedInitialPosition[0] = true;
    }

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d targetPose = targetPoseSupplier.get();

              if (targetPose == null) {
                drive.runVelocity(new ChassisSpeeds(0, 0, 0));
                return;
              }

              // Define initial position and final target
              Pose2d robotEndPose;

              if (!reachedInitialPosition[0]) {
                // Phase 1: Go to position at specified distance from target
                double targetX =
                    targetPose.getX()
                        + Math.cos(targetPose.getRotation().getRadians()) * targetDistance;
                double targetY =
                    targetPose.getY()
                        + Math.sin(targetPose.getRotation().getRadians()) * targetDistance;
                robotEndPose =
                    new Pose2d(
                        targetX, targetY, targetPose.getRotation().plus(new Rotation2d(Math.PI)));
              } else {
                // Phase 2: Go directly to target pose
                // Keep the same rotation (facing the original direction)
                robotEndPose =
                    new Pose2d(
                        targetPose.getX(),
                        targetPose.getY(),
                        targetPose.getRotation().plus(new Rotation2d(Math.PI)));
              }

              // Calculate distance to target position
              double dx = robotEndPose.getX() - currentPose.getX();
              double dy = robotEndPose.getY() - currentPose.getY();
              double distanceToTarget = Math.hypot(dx, dy);

              // Check if we've reached the initial position
              if (!reachedInitialPosition[0] && distanceToTarget < positionThreshold) {
                reachedInitialPosition[0] = true;
                // Reset controllers with current position and velocity (0) when transitioning to
                // phase 2
                drive.xController.reset(currentPose.getX(), 0);
                drive.yController.reset(currentPose.getY(), 0);
              }

              // Log alignment data
              String phase = reachedInitialPosition[0] ? "Final" : "Approach";
              logAlignmentData(currentPose, robotEndPose, phase);
              // Calculate angle error (normalized between -π and π)
              double targetAngle = robotEndPose.getRotation().getRadians();
              double currentAngle = currentPose.getRotation().getRadians();
              double angleError = MathUtil.angleModulus(targetAngle - currentAngle);

              // Adjust blending for more direct movement when far away
              double blendThreshold = 0.000001; // Meters where we start blending
              double autoWeight = Math.min(distanceToTarget / blendThreshold, 1.0);
              double driverWeight = 1.0 - autoWeight;

              // Calculate auto movement speeds using PID
              double xSpeed = drive.xController.calculate(currentPose.getX(), robotEndPose.getX());
              double ySpeed = drive.yController.calculate(currentPose.getY(), robotEndPose.getY());

              // Normalize speeds to avoid exceeding max velocity
              double autoSpeedMagnitude = Math.hypot(xSpeed, ySpeed);
              if (autoSpeedMagnitude > 1.0) {
                xSpeed /= autoSpeedMagnitude;
                ySpeed /= autoSpeedMagnitude;
              }

              // Reduce speed as we get closer to final target
              if (reachedInitialPosition[0] && distanceToTarget < 0.3) {
                double speedScale = distanceToTarget / 0.3; // Scale down speed proportionally
                speedScale = Math.max(0.1, speedScale); // Don't go below 30% speed
                xSpeed *= speedScale;
                ySpeed *= speedScale;
              }

              // Blend driver control with automatic movement
              double finalXSpeed =
                  (forwardSupplier.getAsDouble() * driverWeight) + (xSpeed * autoWeight);
              double finalYSpeed =
                  (strafeSupplier.getAsDouble() * driverWeight) + (ySpeed * autoWeight);

              // Calculate rotation speed using drive's snap controller
              double rotationSpeed = drive.thetaController.calculate(currentAngle, targetAngle);

              if (drive.xController.atGoal()) {
                finalXSpeed = 0;
              }

              if (drive.yController.atGoal()) {
                finalYSpeed = 0;
              }

              if (drive.thetaController.atGoal()) {
                rotationSpeed = 0;
              }

              // Apply speeds to drive
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      finalXSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      finalYSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      rotationSpeed * drive.getMaxAngularSpeedRadPerSec(),
                      currentPose.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              // Reset state when command starts - but keep reachedInitialPosition if we want to go
              // direct
              if (targetDistance != 0) {
                reachedInitialPosition[0] = false;
              }
              Pose2d currentPose = drive.getPose();
              drive.xController.reset(currentPose.getX(), 0);
              drive.yController.reset(currentPose.getY(), 0);
            });
  }

  public static Command alignToTargetLine(
      Drive drive,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      Supplier<Pose2d> targetPoseSupplier) {
    return alignToTargetLine(drive, forwardSupplier, strafeSupplier, targetPoseSupplier, 1.0);
  }

  /**
   * Logs alignment data consistently across different alignment methods.
   *
   * @param methodName Name of the alignment method being used
   * @param drive Drive subsystem
   * @param targetPose Target pose
   * @param xAtSetpoint Whether x position is at setpoint
   * @param yAtSetpoint Whether y position is at setpoint
   * @param rotAtSetpoint Whether rotation is at setpoint
   */
  private static void logAlignmentData(
      String currentPhase,
      Drive drive,
      Pose2d targetPose,
      boolean xAtSetpoint,
      boolean yAtSetpoint,
      boolean rotAtSetpoint) {

    Pose2d currentPose = drive.getPose();

    Logger.recordOutput("AlignToTarget/CurrentPhase", currentPhase);
    Logger.recordOutput("AlignToTarget/TargetPose", targetPose);
    Logger.recordOutput("AlignToTarget/xError", targetPose.getX() - currentPose.getX());
    Logger.recordOutput("AlignToTarget/yError", targetPose.getY() - currentPose.getY());
    Logger.recordOutput(
        "AlignToTarget/RotError",
        MathUtil.angleModulus(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians()));
    Logger.recordOutput("AlignToTarget/xAtSetpoint", xAtSetpoint);
    Logger.recordOutput("AlignToTarget/yAtSetpoint", yAtSetpoint);
    Logger.recordOutput("AlignToTarget/rotAtSetpoint", rotAtSetpoint);
  }

  /**
   * Fine-tunes robot position and rotation using PID controllers. This is designed to be used after
   * path following to achieve precise alignment.
   */
  public static Command pidAlignment(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    // Create PID controllers with appropriate gains
    PIDController xController = new PIDController(3.0, 0.0, 0.1);
    PIDController yController = new PIDController(3.0, 0.0, 0.1);
    PIDController thetaController = new PIDController(4.0, 0.0, 0.2);

    // Enable continuous input for rotation controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for position and rotation
    xController.setTolerance(0.01); // 1 cm
    yController.setTolerance(0.01); // 1 cm
    thetaController.setTolerance(0.02); // ~1 degree

    return Commands.run(
            () -> {
              // Get current pose and target pose
              Pose2d currentPose = drive.getPose();

              if (targetPoseSupplier.get() == null) {
                drive.runVelocity(new ChassisSpeeds());
                return;
              }

              // Create an internal target pose rotated 180 degrees
              /*               Pose2d internalTargetPose =
              new Pose2d(
                  targetPoseSupplier.get().getTranslation(),
                  targetPoseSupplier.get().getRotation().plus(new Rotation2d(Math.PI)));
                   */

              // Calculate PID outputs using the rotated internal target
              double xSpeed =
                  xController.calculate(currentPose.getX(), targetPoseSupplier.get().getX());
              double ySpeed =
                  yController.calculate(currentPose.getY(), targetPoseSupplier.get().getY());
              double rotSpeed =
                  thetaController.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPoseSupplier.get().getRotation().getRadians());

              // Limit speeds for fine adjustment
              double maxLinearSpeed = 1.0; // m/s
              double maxRotSpeed = 2.0; // rad/s

              xSpeed = MathUtil.clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
              ySpeed = MathUtil.clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
              rotSpeed = MathUtil.clamp(rotSpeed, -maxRotSpeed, maxRotSpeed);

              // Log alignment data
              logAlignmentData(
                  "PID Alignment",
                  drive,
                  targetPoseSupplier.get(),
                  xController.atSetpoint(),
                  yController.atSetpoint(),
                  thetaController.atSetpoint());

              // Apply field-relative speeds
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, rotSpeed, currentPose.getRotation()));
            },
            drive)
        .until(
            () ->
                xController.atSetpoint()
                    && yController.atSetpoint()
                    && thetaController.atSetpoint())
        .finallyDo(
            (interrupted) -> {
              // Stop the drive when done
              drive.runVelocity(new ChassisSpeeds());

              // Clean up controllers
              xController.close();
              yController.close();
              thetaController.close();
            });
  }

  public static Command preciseAlignment(
      Drive driveSubsystem, Supplier<Pose2d> preciseTarget, Rotation2d approachDirection) {
    PathConstraints constraints = Constants.DriveToPoseConstraints.slowpathConstraints;

    return Commands.defer(
        () -> {
          if (preciseTarget.get() == null
              || preciseTarget.get().getRotation() == null
              || driveSubsystem.getPose().getRotation() == null) {
            return Commands.none();
          }

          /*           Pose2d rotatedTargetPose =
          new Pose2d(
              preciseTarget.get().getTranslation(),
              preciseTarget.get().getRotation().plus(new Rotation2d(Math.PI)));
              */

          // Log initial alignment data
          logAlignmentData(
              "Path Following",
              driveSubsystem,
              preciseTarget.get(),
              false, // Not at setpoint yet
              false,
              false);

          try {
            // Log alignment data before following path
            logAlignmentData(driveSubsystem.getPose(), preciseTarget.get(), "Precise");
            return AutoBuilder.followPath(
                getPreciseAlignmentPath(
                    constraints,
                    driveSubsystem.getChassisSpeeds(),
                    driveSubsystem.getPose(),
                    preciseTarget.get(),
                    approachDirection));

          } catch (Exception e) {
            return Commands.print("Follow Path Error: " + e.getMessage());
          }
        },
        Set.of(driveSubsystem));
  }

  public static Command preciseAlignmentOld(
      Drive driveSubsystem,
      Supplier<Pose2d> preciseTarget,
      Supplier<Rotation2d> approachDirection) {
    PathConstraints constraints = Constants.DriveToPoseConstraints.slowpathConstraints;

    return Commands.defer(
        () -> {
          if (preciseTarget.get().getRotation() == null
              || driveSubsystem.getPose().getRotation() == null) {
            return Commands.none();
          }
          Logger.recordOutput("PrecisetargetPose", preciseTarget.get());
          // AtomicReference<Rotation2d> preciseTargetRotation2d =
          //     new AtomicReference<>(preciseTarget.get().getRotation());
          try {
            return AutoBuilder.followPath(
                getPreciseAlignmentPath(
                    constraints,
                    driveSubsystem.getChassisSpeeds(),
                    driveSubsystem.getPose(),
                    preciseTarget.get(),
                    approachDirection.get()));
          } catch (Exception e) {
            return Commands.print("Follow Path");
          }
        },
        Set.of(driveSubsystem));
  }

  private static PathPlannerPath getPreciseAlignmentPath(
      PathConstraints constraints,
      ChassisSpeeds measuredSpeedsFieldRelative,
      Pose2d currentRobotPose,
      Pose2d preciseTarget,
      Rotation2d preciseTargetApproachDirection) {
    Translation2d interiorWaypoint = preciseTarget.getTranslation();
    Translation2d fieldRelativeSpeedsMPS =
        new Translation2d(
            measuredSpeedsFieldRelative.vxMetersPerSecond,
            measuredSpeedsFieldRelative.vyMetersPerSecond);
    Rotation2d startingPathDirection =
        fieldRelativeSpeedsMPS
            .times(0.8)
            .plus(interiorWaypoint.minus(currentRobotPose.getTranslation()))
            .getAngle();

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentRobotPose.getTranslation(), startingPathDirection),
            // new Pose2d(interiorWaypoint, preciseTargetApproachDirection),
            new Pose2d(preciseTarget.getTranslation(), preciseTargetApproachDirection));
    List<Waypoint> solitaryWaypoints =
        PathPlannerPath.waypointsFromPoses(
            currentRobotPose,
            new Pose2d(
                currentRobotPose.getTranslation().plus(new Translation2d(1, 1)),
                preciseTarget.getRotation()));
    Waypoint w1 =
        new Waypoint(
            currentRobotPose
                .getTranslation()
                .plus(
                    new Translation2d(
                        .25 * Math.cos(currentRobotPose.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians()))),
            currentRobotPose.getTranslation(),
            currentRobotPose
                .getTranslation()
                .minus(
                    (new Translation2d(
                        .25 * Math.cos(currentRobotPose.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians())))));
    Waypoint w2 =
        new Waypoint(
            preciseTarget
                .getTranslation()
                .plus(
                    new Translation2d(
                        .25 * Math.cos(preciseTarget.getRotation().getRadians()),
                        .25 * Math.sin(preciseTarget.getRotation().getRadians()))),
            preciseTarget.getTranslation(),
            preciseTarget
                .getTranslation()
                .minus(
                    (new Translation2d(
                        .25 * Math.cos(preciseTarget.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians())))));
    List<Waypoint> altwaypoints = List.of(w1, w2);

    List<RotationTarget> rotationTargets =
        List.of(new RotationTarget(1.0, preciseTarget.getRotation()));
    List<ConstraintsZone> constraintsZones =
        List.of(
            new ConstraintsZone(1.0, 2.0, Constants.DriveToPoseConstraints.slowpathConstraints));

    // Logger.recordOutput("DriveCommands/GoalEndState", preciseTargetApproachDirection);
    PathPlannerPath path;

    path =
        new PathPlannerPath(
            waypoints,
            rotationTargets,
            List.of(),
            constraintsZones,
            List.of(),
            constraints,
            new IdealStartingState(
                fieldRelativeSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
            new GoalEndState(MetersPerSecond.of(0), preciseTargetApproachDirection),
            false);

    PathPlannerPath simplerpath =
        new PathPlannerPath(
            solitaryWaypoints,
            rotationTargets,
            List.of(),
            constraintsZones,
            List.of(),
            constraints,
            new IdealStartingState(
                fieldRelativeSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
            new GoalEndState(MetersPerSecond.of(0), preciseTargetApproachDirection),
            false);

    path.preventFlipping = true;
    // path.getIdealTrajectory(drive.getConfig())

    return path;
  }

  public static Command preciseAlignmentsim(
      Drive driveSubsystem,
      Supplier<Pose2d> preciseTarget,
      Supplier<Rotation2d> approachDirection) {
    PathConstraints constraints = Constants.DriveToPoseConstraints.slowpathConstraints;

    return Commands.defer(
        () -> {
          if (preciseTarget.get().getRotation() == null
              || driveSubsystem.getPose().getRotation() == null) {
            return Commands.none();
          }
          Logger.recordOutput("AlignToTarget/TargetPose", preciseTarget.get());
          // AtomicRefer() -> ence<Rotation2d> preciseTargetRotation2d =
          //     new AtomicReference<>(preciseTarget.get().getRotation());
          try {
            return AutoBuilder.followPath(
                getPreciseAlignmentPathsim(
                    constraints,
                    driveSubsystem.getChassisSpeeds(),
                    driveSubsystem.getPose(),
                    preciseTarget.get(),
                    approachDirection.get()));
          } catch (Exception e) {
            return Commands.print("Follow Path");
          }
        },
        Set.of(driveSubsystem));
  }

  private static PathPlannerPath getPreciseAlignmentPathsim(
      PathConstraints constraints,
      ChassisSpeeds measuredSpeedsFieldRelative,
      Pose2d currentRobotPose,
      Pose2d preciseTarget,
      Rotation2d preciseTargetApproachDirection) {
    Translation2d interiorWaypoint = preciseTarget.getTranslation();
    Translation2d fieldRelativeSpeedsMPS =
        new Translation2d(
            measuredSpeedsFieldRelative.vxMetersPerSecond,
            measuredSpeedsFieldRelative.vyMetersPerSecond);
    Rotation2d startingPathDirection =
        fieldRelativeSpeedsMPS
            .times(0.8)
            .plus(interiorWaypoint.minus(currentRobotPose.getTranslation()))
            .getAngle();

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentRobotPose.getTranslation(), startingPathDirection),
            // new Pose2d(interiorWaypoint, preciseTargetApproachDirection),
            new Pose2d(preciseTarget.getTranslation(), preciseTargetApproachDirection));
    List<Waypoint> solitaryWaypoints =
        PathPlannerPath.waypointsFromPoses(
            currentRobotPose,
            new Pose2d(
                currentRobotPose.getTranslation().plus(new Translation2d(1, 1)),
                preciseTarget.getRotation()));
    Waypoint w1 =
        new Waypoint(
            currentRobotPose
                .getTranslation()
                .plus(
                    new Translation2d(
                        .25 * Math.cos(currentRobotPose.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians()))),
            currentRobotPose.getTranslation(),
            currentRobotPose
                .getTranslation()
                .minus(
                    (new Translation2d(
                        .25 * Math.cos(currentRobotPose.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians())))));
    Waypoint w2 =
        new Waypoint(
            preciseTarget
                .getTranslation()
                .plus(
                    new Translation2d(
                        .25 * Math.cos(preciseTarget.getRotation().getRadians()),
                        .25 * Math.sin(preciseTarget.getRotation().getRadians()))),
            preciseTarget.getTranslation(),
            preciseTarget
                .getTranslation()
                .minus(
                    (new Translation2d(
                        .25 * Math.cos(preciseTarget.getRotation().getRadians()),
                        .25 * Math.sin(currentRobotPose.getRotation().getRadians())))));
    List<Waypoint> altwaypoints = List.of(w1, w2);

    List<RotationTarget> rotationTargets =
        List.of(new RotationTarget(1.0, preciseTarget.getRotation()));
    List<ConstraintsZone> constraintsZones =
        List.of(
            new ConstraintsZone(1.0, 2.0, Constants.DriveToPoseConstraints.slowpathConstraints));

    // Logger.recordOutput("DriveCommands/GoalEndState", preciseTargetApproachDirection);
    PathPlannerPath path;

    path =
        new PathPlannerPath(
            altwaypoints,
            rotationTargets,
            List.of(),
            constraintsZones,
            List.of(),
            constraints,
            new IdealStartingState(
                fieldRelativeSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
            new GoalEndState(MetersPerSecond.of(0), preciseTargetApproachDirection),
            false);

    PathPlannerPath simplerpath =
        new PathPlannerPath(
            solitaryWaypoints,
            rotationTargets,
            List.of(),
            constraintsZones,
            List.of(),
            constraints,
            new IdealStartingState(
                fieldRelativeSpeedsMPS.getNorm(), currentRobotPose.getRotation()),
            new GoalEndState(MetersPerSecond.of(0), preciseTargetApproachDirection),
            false);

    path.preventFlipping = true;
    // path.getIdealTrajectory(drive.getConfig())

    return path;
  }

  public static Command driveToPoseStraight(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    PIDController xSpeedController = new PIDController(5, 0, 0.1);
    PIDController ySpeedController = new PIDController(5, 0, 0.1);
    PIDController angularSpeedController = new PIDController(3, 0, 0.025);
    return Commands.run(
            () -> {
              Pose2d currentPose = drive.getPose();
              Pose2d targetPose = targetPoseSupplier.get();
              Rotation2d currentRotation = currentPose.getRotation();
              Rotation2d targetRotation = targetPose.getRotation();

              double fieldVx = xSpeedController.calculate(currentPose.getX(), targetPose.getX());
              double fieldVy = ySpeedController.calculate(currentPose.getY(), targetPose.getY());

              ChassisSpeeds robotRelativeChassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldVx,
                      fieldVy,
                      angularSpeedController.calculate(
                          currentRotation.getRadians(), targetRotation.getRadians()),
                      currentPose.getRotation());

              drive.runVelocity(robotRelativeChassisSpeeds);
            })
        .beforeStarting(
            () -> {
              xSpeedController.reset();
              ySpeedController.reset();
              angularSpeedController.reset();
            })
        .finallyDo(
            () -> {
              xSpeedController.close();
              ySpeedController.close();
              angularSpeedController.close();
            });
  }

  /**
   * Creates a command that continuously logs alignment data for a given target pose. This is useful
   * for monitoring alignment status even when not actively aligning.
   */
  public static Command logAlignmentPeriodically(Drive drive, Supplier<Pose2d> targetPose) {
    return Commands.run(
        () -> {
          Pose2d currentPose = drive.getPose();
          Pose2d target = targetPose.get();
          if (target != null) {
            logAlignmentData(currentPose, target, "Monitoring");
          }
        },
        drive);
  }
}
