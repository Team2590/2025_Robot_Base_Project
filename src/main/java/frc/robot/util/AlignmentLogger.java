package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class AlignmentLogger {
    private static final double X_ALIGNMENT_TOLERANCE = 0.05;  // meters
    private static final double Y_ALIGNMENT_TOLERANCE = 0.05;  // meters
    private static final double ROTATION_ALIGNMENT_TOLERANCE = Units.degreesToRadians(3.0);  // 3 degrees in radians

    public record AlignmentStatus(
        boolean xAligned, 
        boolean yAligned, 
        boolean rotationAligned, 
        boolean fullyAligned) {}

    public static AlignmentStatus checkAlignmentTolerances(Pose2d currentPose, Pose2d targetPose) {
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        boolean xAligned = Math.abs(xError) <= X_ALIGNMENT_TOLERANCE;
        boolean yAligned = Math.abs(yError) <= Y_ALIGNMENT_TOLERANCE;
        boolean rotationAligned = Math.abs(rotationError) <= ROTATION_ALIGNMENT_TOLERANCE;
        boolean fullyAligned = xAligned && yAligned && rotationAligned;

        return new AlignmentStatus(xAligned, yAligned, rotationAligned, fullyAligned);
    }

    public static void logAlignmentData(String prefix, Pose2d currentPose, Pose2d targetPose, String phase) {
        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        // Get alignment status
        AlignmentStatus alignmentStatus = checkAlignmentTolerances(currentPose, targetPose);

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
        Logger.recordOutput(prefix + "/Tolerances/X_Meters", X_ALIGNMENT_TOLERANCE);
        Logger.recordOutput(prefix + "/Tolerances/Y_Meters", Y_ALIGNMENT_TOLERANCE);
        Logger.recordOutput(prefix + "/Tolerances/Rotation_Degrees", 
            Units.radiansToDegrees(ROTATION_ALIGNMENT_TOLERANCE));
    }
} 