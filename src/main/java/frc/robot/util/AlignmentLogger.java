package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AlignmentLogger {
  public record AlignmentStatus(
      boolean xAligned, boolean yAligned, boolean rotationAligned, boolean fullyAligned) {}

  public static AlignmentStatus checkAlignmentTolerances(Pose2d currentPose, Pose2d targetPose) {

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError =
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    double xTolerance = Constants.DriveToPoseStraight.XController.tolerance;
    double yTolerance = Constants.DriveToPoseStraight.YController.tolerance;
    double rotationTolerance = Constants.DriveToPoseStraight.ThetaController.tolerance;

    boolean xAligned = Math.abs(xError) <= xTolerance;
    boolean yAligned = Math.abs(yError) <= yTolerance;
    boolean rotationAligned = Math.abs(rotationError) <= rotationTolerance;
    boolean fullyAligned = xAligned && yAligned && rotationAligned;

    return new AlignmentStatus(xAligned, yAligned, rotationAligned, fullyAligned);
  }

  public static void logAlignmentData(String prefix, Pose2d currentPose, Pose2d targetPose) {

    // Calculate errors
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError =
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Get alignment status
    AlignmentStatus alignmentStatus = checkAlignmentTolerances(currentPose, targetPose);

    // Log essential data
    Logger.recordOutput(prefix + "/CurrentPose", currentPose);
    Logger.recordOutput(prefix + "/TargetPose", targetPose);

    // Log errors in absolute units
    Logger.recordOutput(prefix + "/Errors/X_Meters", xError);
    Logger.recordOutput(prefix + "/Errors/Y_Meters", yError);
    Logger.recordOutput(prefix + "/Errors/Rotation_Degrees", Units.radiansToDegrees(rotationError));

    // Log alignment status
    Logger.recordOutput(prefix + "/Aligned/X", alignmentStatus.xAligned());
    Logger.recordOutput(prefix + "/Aligned/Y", alignmentStatus.yAligned());
    Logger.recordOutput(prefix + "/Aligned/Rotation", alignmentStatus.rotationAligned());
    Logger.recordOutput(prefix + "/Aligned/Full", alignmentStatus.fullyAligned());
  }
}
