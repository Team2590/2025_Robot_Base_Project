package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignmentLogger;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class NemesisDriveToPoseStraight extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final String prefix = "NemesisDriveToPoseStraight";

  public NemesisDriveToPoseStraight(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;

    // Create PID controllers with appropriate gains
    this.xController = new PIDController(Constants.DriveToPoseStraight.XController.kP, 0.0, Constants.DriveToPoseStraight.XController.kD);
    this.yController = new PIDController(Constants.DriveToPoseStraight.YController.kP, 0.0, Constants.DriveToPoseStraight.YController.kD);
    this.thetaController = new PIDController(Constants.DriveToPoseStraight.ThetaController.kP, 0.0, Constants.DriveToPoseStraight.ThetaController.kD);

    // Enable continuous input for rotation controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for position and rotation
    xController.setTolerance(Constants.DriveToPoseStraight.XController.tolerance);
    yController.setTolerance(Constants.DriveToPoseStraight.YController.tolerance);
    thetaController.setTolerance(Constants.DriveToPoseStraight.ThetaController.tolerance);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset controllers without parameters
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setPID(
      Drive.xControllerP.get(),
    0,
    Constants.DriveToPoseStraight.XController.kD);
    xController.setTolerance(Constants.DriveToPoseStraight.XController.tolerance);

    yController.setPID(
      Drive.yControllerP.get(),
    0,
    Constants.DriveToPoseStraight.YController.kD);
    yController.setTolerance(Constants.DriveToPoseStraight.YController.tolerance);

    thetaController.setPID(
      Drive.ThetaConstrollerP.get(),
    0,
    Constants.DriveToPoseStraight.ThetaController.kD);
    thetaController.setTolerance(
    Constants.DriveToPoseStraight.ThetaController.tolerance);
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
    double maxLinearSpeed = Constants.DriveToPoseStraight.maxVelocityMPS; // m/s
    double maxRotSpeed = Constants.DriveToPoseStraight.maxAngularVelocityRadPerSec; // rad/s

    xSpeed = MathUtil.clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
    rotSpeed = MathUtil.clamp(rotSpeed, -maxRotSpeed, maxRotSpeed);

    // Use AlignmentLogger for logging
    AlignmentLogger.logAlignmentData(
        prefix, 
        currentPose, 
        targetPose);

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
        currentPose, targetPose).fullyAligned();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    xController.close();
    yController.close();
    thetaController.close();
  }
}