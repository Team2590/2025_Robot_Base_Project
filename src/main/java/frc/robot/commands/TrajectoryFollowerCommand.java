package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.NemesisHolonomicDriveController;
import frc.robot.util.AlignmentLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.AlignmentConstants.ToleranceMode;

public class TrajectoryFollowerCommand extends Command {

  private Supplier<PathPlannerPath> pathSupplier;
  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private final boolean isInitialPoint;
  private final Drive drive;
  private final Timer timer = new Timer();
  private boolean flippedForRed = false;
  private boolean firstTime = false;
  private ChassisSpeeds startingSpeeds;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private Rotation2d endGoal;
  private boolean runOnce = false;
  private Pose2d finalTargetPose = null;
  private boolean initializationComplete = false;
  private final ToleranceMode toleranceMode;

  public NemesisHolonomicDriveController autonomusController =
      new NemesisHolonomicDriveController(
          new PIDController(8, 0, 0.0), new PIDController(8, 0, 0.0), new PIDController(6, 0, .2));

  ProfiledPIDController angleController =
      new ProfiledPIDController(
          6,
          0.0,
          0.2,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      Rotation2d endGoalRotation,
      boolean isInitialPoint,
      ChassisSpeeds startingSpeeds,
      ToleranceMode toleranceMode) {
    this.pathSupplier = path;
    this.drive = drive;
    this.isInitialPoint = isInitialPoint;
    this.startingSpeeds = startingSpeeds;
    this.endGoal = endGoalRotation;
    this.toleranceMode = toleranceMode;
    addRequirements(drive);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      Rotation2d endGoalRotation,
      boolean isInitialPoint,
      ChassisSpeeds startingSpeeds) {
    this(path, drive, endGoalRotation, isInitialPoint, startingSpeeds, ToleranceMode.STRICT);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      Rotation2d endGoalRotation,
      boolean isInitialPoint,
      Supplier<Boolean> aimAtSpeaker) {
    this(path, drive, endGoalRotation, isInitialPoint, new ChassisSpeeds());
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path, Rotation2d endGoalRotation, Drive swerve) {
    this(path, swerve, endGoalRotation, false, new ChassisSpeeds());
  }

  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(.01);
    timer.reset();
    timer.start();
    initializationComplete = false;

    try {
      if (!firstTime) {
        path = pathSupplier.get();
        firstTime = true;
      }
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red
          && !flippedForRed) {
        // path = path.flipPath();
        flippedForRed = true;
      }

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Blue
          && flippedForRed) {
        path = path.flipPath();
        flippedForRed = false;
      }
      if (isInitialPoint) {
        trajectory =
            path.generateTrajectory(
                new ChassisSpeeds(),
                path.getStartingHolonomicPose().get().getRotation(),
                drive.getConfig());

        Logger.recordOutput(
            "TrajectoryFollower/InitialHolonomicPose", path.getStartingHolonomicPose().get());
        drive.setPose(path.getStartingHolonomicPose().get());
      } else {
        trajectory =
            path.generateTrajectory(
                startingSpeeds, drive.getPose().getRotation(), drive.getConfig());
      }

      // Store the final target pose
      finalTargetPose = new Pose2d(trajectory.getEndState().pose.getTranslation(), endGoal);

      initializationComplete = true;

      // Log initial state
      Logger.recordOutput("TrajectoryFollower/Initialization/Complete", true);
      Logger.recordOutput("TrajectoryFollower/FinalTargetPose", finalTargetPose);
    } catch (Exception e) {
      Logger.recordOutput("TrajectoryFollower/Initialization/Error", true);
      finalTargetPose = null;
      initializationComplete = false;
    }

    List<Pose2d> poses = new ArrayList<>();

    for (PathPlannerTrajectoryState state : trajectory.getStates()) {
      poses.add(state.pose);
    }

    Pose2d[] Poses = new Pose2d[trajectory.getStates().size()];

    for (int i = 0; i < poses.size(); i++) {
      Poses[i] = poses.get(i);
    }
    Logger.recordOutput("TrajectoryFollower/Poses", Poses);
    Logger.recordOutput("TrajectoryFollower/finalPose", Poses[Poses.length - 1]);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    try {
      PathPlannerTrajectoryState goal = trajectory.sample(timer.get());
      ChassisSpeeds adjustedSpeeds = autonomusController.calculate(drive.getPose(), goal);
      drive.runVelocity(adjustedSpeeds);

      // Use AlignmentLogger for logging
      AlignmentLogger.logAlignmentData(
          "TrajectoryFollower",
          drive.getPose(),
          finalTargetPose,
          timer.get() >= trajectory.getTotalTimeSeconds() ? "Final Rotation" : "Path Following",
          toleranceMode);

      if (timer.get() >= trajectory.getTotalTimeSeconds()) {
        if (!runOnce) {
          runOnce = true;
          angleController.reset(drive.getPose().getRotation().getRadians());
        }
        double omega =
            angleController.calculate(
                drive.getPose().getRotation().getRadians(), endGoal.getRadians());

        Logger.recordOutput(
            "TrajectoryFollower/angleControllerError", angleController.getPositionError());
        Logger.recordOutput(
            "TrajectoryFollower/angleControllerAtSetpoint", angleController.atSetpoint());

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
      }
    } catch (Exception e) {
      Logger.recordOutput("TrajectoryFollower/Error/ExecuteError", true);
      return;
    }
  }

  @Override
  public boolean isFinished() {
    if (this.trajectory == null || finalTargetPose == null) {
      return true;
    }

    if (timer.get() >= trajectory.getTotalTimeSeconds()) {
      return AlignmentLogger.checkAlignmentTolerances(drive.getPose(), finalTargetPose, toleranceMode).fullyAligned();
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
