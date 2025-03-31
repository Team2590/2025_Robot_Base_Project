package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentLogger;
import java.util.function.Supplier;
import frc.robot.Constants.AlignmentConstants.ToleranceMode;

public class PIDAlignmentCommand extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final ToleranceMode toleranceMode;

  public PIDAlignmentCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier, ToleranceMode toleranceMode) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.toleranceMode = toleranceMode;

    // Create PID controllers with appropriate gains
    this.xController = new PIDController(8.0, 0.0, 0.0);
    this.yController = new PIDController(8.0, 0.0, 0.0);
    this.thetaController = new PIDController(6.0, 0.0, 0.2);

    // Enable continuous input for rotation controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for position and rotation
    xController.setTolerance(0.01); // 1 cm
    yController.setTolerance(0.01); // 1 cm
    thetaController.setTolerance(0.02); // ~1 degree

    addRequirements(drive);
  }

  // Overload constructor with default STRICT tolerance
  public PIDAlignmentCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, ToleranceMode.STRICT);
  }

  @Override
  public void initialize() {
    // Reset controllers without parameters
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    if (targetPose == null) {
      drive.runVelocity(new ChassisSpeeds());
      return;
    }

    // Calculate PID outputs
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotSpeed =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Limit speeds for fine adjustment
    double maxLinearSpeed = 1.0; // m/s
    double maxRotSpeed = 2.0; // rad/s

    xSpeed = MathUtil.clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
    rotSpeed = MathUtil.clamp(rotSpeed, -maxRotSpeed, maxRotSpeed);

    // Use AlignmentLogger for logging
    AlignmentLogger.logAlignmentData(
        "PID Alignment", 
        currentPose, 
        targetPose, 
        "Final Alignment",
        toleranceMode);

    // Apply field-relative speeds
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, currentPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    if (targetPose == null) {
      return true;
    }

    return AlignmentLogger.checkAlignmentTolerances(
        currentPose, targetPose, toleranceMode).fullyAligned();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    xController.close();
    yController.close();
    thetaController.close();
  }
}