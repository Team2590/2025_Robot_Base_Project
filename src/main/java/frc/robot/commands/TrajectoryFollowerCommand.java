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
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
  private static final double X_ALIGNMENT_TOLERANCE = 0.05; // meters
  private static final double Y_ALIGNMENT_TOLERANCE = 0.05; // meters
  private static final double ROTATION_ALIGNMENT_TOLERANCE = 0.05; // radians
  private static final double ALIGNMENT_STABLE_TIME = 0.1; // seconds
  private double lastAlignmentTime = 0;
  private boolean wasAligned = false;
  private Pose2d finalTargetPose = null;
  private boolean initializationComplete = false;

  public final NemesisHolonomicDriveController autonomusController =
      new NemesisHolonomicDriveController(
          new PIDController(8.0, 0, 0.0),
          new PIDController(8.0, 0, 0.0),
          new PIDController(6.0, 0, 0.2));

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
      ChassisSpeeds startingSpeeds) {
    this.pathSupplier = path;
    this.drive = drive;
    this.isInitialPoint = isInitialPoint;
    this.startingSpeeds = startingSpeeds;
    this.endGoal = endGoalRotation;
    addRequirements(drive);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      Rotation2d endGoalRotation,
      ChassisSpeeds startingSpeeds) {
    this(path, drive, endGoalRotation, false, startingSpeeds);
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
    timer.reset();
    timer.start();
    wasAligned = false;
    lastAlignmentTime = 0;
  }

  @Override
  public void execute() {
    try {
      PathPlannerTrajectoryState goal = trajectory.sample(timer.get());
      ChassisSpeeds adjustedSpeeds = autonomusController.calculate(drive.getPose(), goal);
      drive.runVelocity(adjustedSpeeds);

      // Log trajectory data
      logTrajectoryData(
          drive.getPose(),
          timer.get() >= trajectory.getTotalTimeSeconds() ? "Final Rotation" : "Path Following");

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

  private record AlignmentStatus(
      boolean xAligned, boolean yAligned, boolean rotationAligned, boolean fullyAligned) {}

  private AlignmentStatus checkAlignmentTolerances(Pose2d currentPose, Pose2d targetPose) {
    // Calculate errors
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError =
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Log raw error calculations
    Logger.recordOutput("TrajectoryFollower/Debug/RawErrors/X", xError);
    Logger.recordOutput("TrajectoryFollower/Debug/RawErrors/Y", yError);
    Logger.recordOutput("TrajectoryFollower/Debug/RawErrors/Rotation", rotationError);

    // Log tolerance checks
    boolean xAligned = Math.abs(xError) <= X_ALIGNMENT_TOLERANCE;
    boolean yAligned = Math.abs(yError) <= Y_ALIGNMENT_TOLERANCE;
    boolean rotationAligned = Math.abs(rotationError) <= ROTATION_ALIGNMENT_TOLERANCE;

    // Log individual tolerance results
    Logger.recordOutput("TrajectoryFollower/Debug/ToleranceChecks/X_Error", Math.abs(xError));
    Logger.recordOutput("TrajectoryFollower/Debug/ToleranceChecks/Y_Error", Math.abs(yError));
    Logger.recordOutput(
        "TrajectoryFollower/Debug/ToleranceChecks/Rotation_Error", Math.abs(rotationError));
    Logger.recordOutput(
        "TrajectoryFollower/Debug/ToleranceChecks/X_Tolerance", X_ALIGNMENT_TOLERANCE);
    Logger.recordOutput(
        "TrajectoryFollower/Debug/ToleranceChecks/Y_Tolerance", Y_ALIGNMENT_TOLERANCE);
    Logger.recordOutput(
        "TrajectoryFollower/Debug/ToleranceChecks/Rotation_Tolerance",
        ROTATION_ALIGNMENT_TOLERANCE);

    boolean fullyAligned = xAligned && yAligned && rotationAligned;

    // Log the inputs used for this calculation
    Logger.recordOutput("TrajectoryFollower/Debug/CheckInputs/CurrentPose", currentPose);
    Logger.recordOutput("TrajectoryFollower/Debug/CheckInputs/TargetPose", targetPose);

    return new AlignmentStatus(xAligned, yAligned, rotationAligned, fullyAligned);
  }

  private void logTrajectoryData(Pose2d currentPose, String phase) {
    // Calculate errors
    double xError = finalTargetPose.getX() - currentPose.getX();
    double yError = finalTargetPose.getY() - currentPose.getY();
    double rotationError =
        MathUtil.angleModulus(
            finalTargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Get alignment status
    AlignmentStatus alignmentStatus = checkAlignmentTolerances(currentPose, finalTargetPose);

    // Log essential data
    Logger.recordOutput("TrajectoryFollower/Phase", phase);
    Logger.recordOutput("TrajectoryFollower/CurrentPose", currentPose);
    Logger.recordOutput("TrajectoryFollower/FinalTargetPose", finalTargetPose);

    // Log errors in absolute units
    Logger.recordOutput("TrajectoryFollower/Errors/X_Meters", xError);
    Logger.recordOutput("TrajectoryFollower/Errors/Y_Meters", yError);
    Logger.recordOutput(
        "TrajectoryFollower/Errors/Rotation_Degrees", Units.radiansToDegrees(rotationError));

    // Log alignment status
    Logger.recordOutput("TrajectoryFollower/Aligned/X", alignmentStatus.xAligned());
    Logger.recordOutput("TrajectoryFollower/Aligned/Y", alignmentStatus.yAligned());
    Logger.recordOutput("TrajectoryFollower/Aligned/Rotation", alignmentStatus.rotationAligned());
    Logger.recordOutput("TrajectoryFollower/Aligned/Full", alignmentStatus.fullyAligned());

    // Log tolerances for reference
    Logger.recordOutput("TrajectoryFollower/Tolerances/X_Meters", X_ALIGNMENT_TOLERANCE);
    Logger.recordOutput("TrajectoryFollower/Tolerances/Y_Meters", Y_ALIGNMENT_TOLERANCE);
    Logger.recordOutput(
        "TrajectoryFollower/Tolerances/Rotation_Degrees",
        Units.radiansToDegrees(ROTATION_ALIGNMENT_TOLERANCE));
  }

  @Override
  public boolean isFinished() {
    boolean finished = false;

    if (this.trajectory == null || finalTargetPose == null) {
      finished = true;
    } else if (timer.get() >= trajectory.getTotalTimeSeconds()) {
      finished = checkAlignmentTolerances(drive.getPose(), finalTargetPose).fullyAligned();
    }

    // Log isFinished status
    Logger.recordOutput("TrajectoryFollower/isFinished", finished);
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
