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
import frc.robot.RobotState;
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

  private DriveCommands() {}

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
    return AutoBuilder.pathfindToPose(targetPose, DriveToPoseConstraints.fastpathConstraints, 0.0);
  }

  public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
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

  public static Command preciseAlignmentAutoBuilder(
      Drive driveSubsystem, Supplier<Pose2d> preciseTarget, Rotation2d approachDirection) {

    return Commands.defer(
        () -> {
          PathConstraints constraints = Constants.DriveToPoseConstraints.slowpathConstraints;
          if (preciseTarget.get().getRotation() == null
              || driveSubsystem.getPose().getRotation() == null) {
            return Commands.none();
          }
          try {
            Command pathCommand =
                AutoBuilder.followPath(
                    getPreciseAlignmentPath(
                        constraints,
                        driveSubsystem.getChassisSpeeds(),
                        driveSubsystem.getPose(),
                        preciseTarget.get(),
                        approachDirection));
            return wrapForAligning(pathCommand, preciseTarget);
          } catch (Exception e) {
            RobotState.getInstance().resetAligningState();
            return Commands.print("Follow Path Exception: " + e.getMessage());
          }
        },
        Set.of(driveSubsystem));
  }

  /**
   * Fine-tunes robot position and rotation using PID controllers. This is designed to be used after
   * path following to achieve precise alignment.
   */
  public static Command pidAlignment(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    return new NemesisDriveToPoseStraight(drive, targetPoseSupplier);
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
            new Pose2d(interiorWaypoint, preciseTargetApproachDirection),
            new Pose2d(preciseTarget.getTranslation(), preciseTarget.getRotation()));

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
            new GoalEndState(MetersPerSecond.of(0), preciseTarget.getRotation()),
            false);

    path.preventFlipping = true;
    // path.getIdealTrajectory(drive.getConfig())

    return path;
  }

  public static Command driveToPoseStraight(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    PIDController xSpeedController =
        new PIDController(
            Constants.DriveToPoseStraight.XController.kP,
            0,
            Constants.DriveToPoseStraight.XController.kD);
    PIDController ySpeedController =
        new PIDController(
            Constants.DriveToPoseStraight.YController.kP,
            0,
            Constants.DriveToPoseStraight.YController.kD);
    PIDController angularSpeedController =
        new PIDController(
            Constants.DriveToPoseStraight.ThetaController.kP,
            0,
            Constants.DriveToPoseStraight.ThetaController.kD);

    xSpeedController.setTolerance(Constants.DriveToPoseStraight.XController.tolerance);
    ySpeedController.setTolerance(Constants.DriveToPoseStraight.YController.tolerance);
    angularSpeedController.setTolerance(Constants.DriveToPoseStraight.ThetaController.tolerance);

    angularSpeedController.enableContinuousInput(-Math.PI, -Math.PI);

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

              xSpeedController.setPID(
                  Constants.DriveToPoseStraight.XController.kP,
                  0,
                  Constants.DriveToPoseStraight.XController.kD);
              xSpeedController.setTolerance(Constants.DriveToPoseStraight.XController.tolerance);

              ySpeedController.setPID(
                  Constants.DriveToPoseStraight.YController.kP,
                  0,
                  Constants.DriveToPoseStraight.YController.kD);
              ySpeedController.setTolerance(Constants.DriveToPoseStraight.YController.tolerance);

              angularSpeedController.setPID(
                  Constants.DriveToPoseStraight.ThetaController.kP,
                  0,
                  Constants.DriveToPoseStraight.ThetaController.kD);
              angularSpeedController.setTolerance(
                  Constants.DriveToPoseStraight.ThetaController.tolerance);
            })
        .finallyDo(
            () -> {
              xSpeedController.close();
              ySpeedController.close();
              angularSpeedController.close();
            })
        .until(
            () ->
                xSpeedController.atSetpoint()
                    && ySpeedController.atSetpoint()
                    && angularSpeedController.atSetpoint());
  }
  // spotless:on

  public static Command joystickDriveToCoral(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive, xSupplier, ySupplier, () -> RobotContainer.getVision().getNearestCoralRotation());
  }
  /**
   * Convenient method that sets the AligningState before running the command and resets it after.
   */
  private static Command wrapForAligning(Command command, Supplier<Pose2d> preciseTarget) {
    RobotState robotState = RobotState.getInstance();
    return command
        .beforeStarting(
            () -> {
              robotState.setAligningStateBasedOnTargetPose(preciseTarget);
            })
        .finallyDo(robotState::resetAligningState);
  }
}
