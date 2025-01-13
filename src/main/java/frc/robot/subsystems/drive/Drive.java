// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  @AutoLogOutput(key = "Drive/VelocityMode")
  private boolean velocityMode = false;

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    swerveSetpointGenerator =
        new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    // Call periodic on modules
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
    }

    // Send odometry updates to robot state
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryObservation(
              new RobotState.OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null),
                  sampleTimestamps[i]));
    }

    // Update current setpoint if not in velocity mode
    if (!velocityMode) {
      currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    velocityMode = true;
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint,
            discreteSpeeds,
            Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * Runs the drive at the desired velocity with setpoint module forces.
   *
   * @param speeds Speeds in meters/sec
   * @param moduleForces The forces applied to each module
   */
  public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
    velocityMode = true;
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint,
            discreteSpeeds,
            Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Save module forces to swerve states for logging
    SwerveModuleState[] wheelForces = new SwerveModuleState[4];
    // Send setpoints to modules
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < 4; i++) {
      // Optimize state
      Rotation2d wheelAngle = moduleStates[i].angle;
      setpointStates[i].optimize(wheelAngle);
      setpointStates[i].cosineScale(wheelAngle);

      // Calculate wheel torque in direction
      var wheelForce = moduleForces.get(i);
      Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
      double wheelTorqueNm = wheelForce.dot(wheelDirection) * DriveConstants.wheelRadius;
      modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

      // Save to array for logging
      wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
    }
    Logger.recordOutput("SwerveStates/ModuleForces", wheelForces);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the raw gyro rotation read by the IMU */
  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.maxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.driveBaseRadius;
  }
}
