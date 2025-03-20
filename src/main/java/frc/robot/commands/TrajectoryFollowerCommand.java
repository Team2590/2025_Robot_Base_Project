package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.NemesisHolonomicDriveController;
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
  private Supplier<Boolean> aimAtSpeaker = () -> false;
  private boolean firstTime = false;
  private ChassisSpeeds startingSpeeds;

  public final NemesisHolonomicDriveController autonomusController =
      new NemesisHolonomicDriveController(
          new PIDController(8.0, 0, 0.0),
          new PIDController(8.0, 0, 0.0),
          new PIDController(7.0, 0, 0.2));

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      boolean isInitialPoint,
      ChassisSpeeds startingSpeeds) {
    this.pathSupplier = path;
    this.drive = drive;
    this.isInitialPoint = isInitialPoint;
    this.startingSpeeds = startingSpeeds;
    addRequirements(drive);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path, Drive drive, ChassisSpeeds startingSpeeds) {
    this(path, drive, false, startingSpeeds);
  }

  public TrajectoryFollowerCommand(
      Supplier<PathPlannerPath> path,
      Drive drive,
      boolean isInitialPoint,
      Supplier<Boolean> aimAtSpeaker) {
    this(path, drive, isInitialPoint, new ChassisSpeeds());
  }

  public TrajectoryFollowerCommand(Supplier<PathPlannerPath> path, Drive swerve) {
    this(path, swerve, false, new ChassisSpeeds());
  }

  @Override
  public void initialize() {
    // Logger.recordOutput("TrajectoryFollowerCommandAlliance", Robot.alliance);
    if (!firstTime) {
      path = pathSupplier.get();
      firstTime = true;
    }
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red
        && !flippedForRed) {
      path = path.flipPath();
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
          path.generateTrajectory(startingSpeeds, drive.getPose().getRotation(), drive.getConfig());
    }
    timer.reset();
    timer.start();
    System.out.println("\n\n\n\nTrajectory time seconds:" + trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    PathPlannerTrajectoryState goal = trajectory.sample(timer.get());
    // goal.targetHolonomicRotation =
    //     aimAtSpeaker.get()
    //         ? RobotContainer.visionSupplier.robotToSpeakerAngleAuto()
    //         : goal.targetHolonomicRotation;
    ChassisSpeeds adjustedSpeeds = autonomusController.calculate(drive.getPose(), goal);
    drive.runVelocity(adjustedSpeeds);

    // Logger.recordOutput(
    //     "TrajectoryFollower/TrajectoryGoal",
    //     new Pose2d(goal.positionMeters, goal.targetHolonomicRotation));
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
