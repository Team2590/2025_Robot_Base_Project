package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AlignmentConstants.ToleranceMode;
import org.littletonrobotics.junction.Logger;

public class AlignmentLogger {
    public record AlignmentStatus(
        boolean xAligned, 
        boolean yAligned, 
        boolean rotationAligned, 
        boolean fullyAligned) {}

    public static AlignmentStatus checkAlignmentTolerances(
            Pose2d currentPose, 
            Pose2d targetPose, 
            ToleranceMode toleranceMode) {
        
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        double xTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_X_TOLERANCE : AlignmentConstants.LOOSE_X_TOLERANCE;
        double yTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_Y_TOLERANCE : AlignmentConstants.LOOSE_Y_TOLERANCE;
        double rotationTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_ROTATION_TOLERANCE : AlignmentConstants.LOOSE_ROTATION_TOLERANCE;

        boolean xAligned = Math.abs(xError) <= xTolerance;
        boolean yAligned = Math.abs(yError) <= yTolerance;
        boolean rotationAligned = Math.abs(rotationError) <= rotationTolerance;
        boolean fullyAligned = xAligned && yAligned && rotationAligned;

        return new AlignmentStatus(xAligned, yAligned, rotationAligned, fullyAligned);
    }

    // Overload for backward compatibility, defaults to STRICT mode
    public static AlignmentStatus checkAlignmentTolerances(Pose2d currentPose, Pose2d targetPose) {
        return checkAlignmentTolerances(currentPose, targetPose, ToleranceMode.STRICT);
    }

    public static void logAlignmentData(
            String prefix, 
            Pose2d currentPose, 
            Pose2d targetPose, 
            String phase,
            ToleranceMode toleranceMode) {
        
        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        // Get alignment status
        AlignmentStatus alignmentStatus = checkAlignmentTolerances(currentPose, targetPose, toleranceMode);

        // Log essential data
        Logger.recordOutput(prefix + "/Phase", phase);
        Logger.recordOutput(prefix + "/CurrentPose", currentPose);
        Logger.recordOutput(prefix + "/TargetPose", targetPose);
        
        // Log errors in absolute units
        Logger.recordOutput(prefix + "/Errors/X_Meters", xError);
        Logger.recordOutput(prefix + "/Errors/Y_Meters", yError);
        Logger.recordOutput(prefix + "/Errors/Rotation_Degrees", 
            Units.radiansToDegrees(rotationError));
        
        // Log alignment status
        Logger.recordOutput(prefix + "/Aligned/X", alignmentStatus.xAligned());
        Logger.recordOutput(prefix + "/Aligned/Y", alignmentStatus.yAligned());
        Logger.recordOutput(prefix + "/Aligned/Rotation", alignmentStatus.rotationAligned());
        Logger.recordOutput(prefix + "/Aligned/Full", alignmentStatus.fullyAligned());
        
        // Log tolerances for reference
        double xTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_X_TOLERANCE : AlignmentConstants.LOOSE_X_TOLERANCE;
        double yTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_Y_TOLERANCE : AlignmentConstants.LOOSE_Y_TOLERANCE;
        double rotationTolerance = (toleranceMode == ToleranceMode.STRICT) ? 
            AlignmentConstants.STRICT_ROTATION_TOLERANCE : AlignmentConstants.LOOSE_ROTATION_TOLERANCE;

        Logger.recordOutput(prefix + "/Tolerances/X_Meters", xTolerance);
        Logger.recordOutput(prefix + "/Tolerances/Y_Meters", yTolerance);
        Logger.recordOutput(prefix + "/Tolerances/Rotation_Degrees", 
            Units.radiansToDegrees(rotationTolerance));
    }

    // Overload for backward compatibility, defaults to STRICT mode
    public static void logAlignmentData(String prefix, Pose2d currentPose, Pose2d targetPose, String phase) {
        logAlignmentData(prefix, currentPose, targetPose, phase, ToleranceMode.STRICT);
    }
} 