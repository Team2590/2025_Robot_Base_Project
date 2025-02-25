package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CoralPose;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;

public class FieldConstants {
  public static enum ZONES {
    REEF,
    BARGE,
    PROCCESOR,
    NOTE
  }

  public static double[] Reef_x = {2.16, 6.61, 2.16, 6.61};
  public static double[] Reef_y = {6.08, 6.08, 1.57, 1.56};
  public static Path2D ReefBounds = create_zone(Reef_x, Reef_y);
  // public static FRCPolygon ReefPolygon= new FRCPolygon("REEF", ReefBounds,false);

  public static double[] Barge_x = {10.23, 10.23, 7.367, 7.367};

  public static Path2D BargeBoundsBLUE = create_zone(Reef_x, Reef_y); // bottom
  public static Path2D BargeBoundsRED =
      create_zone(Reef_x, new double[] {3.5, 4.16, 3.5, .416}); // top

  public static Path2D create_zone(double[] x, double[] y) {

    Path2D bounds = new Path2D.Double();
    bounds.moveTo(x[0], y[0]);

    for (int i = 1; i < x.length; ++i) {
      bounds.lineTo(x[i], y[i]);
    }

    return bounds;
  }

  public static Path2D flipZone(Path2D zone) {
    AffineTransform t = new AffineTransform();
    t = AffineTransform.getQuadrantRotateInstance(2, 8.785, 4.02);
    Shape flipped = zone.createTransformedShape(t);

    return new Path2D.Double(flipped);
  }

  public static class BlueReefPoses {
    public static final Pose2d N_right = CoralPose.getReefPose(21)[0]; // right
    public static final Pose2d N_left = CoralPose.getReefPose(21)[1]; // left
    public static final Pose2d NE_right = CoralPose.getReefPose(22)[0]; // right
    public static final Pose2d NE_left = CoralPose.getReefPose(22)[1]; // left
    public static final Pose2d SE_right = CoralPose.getReefPose(17)[0]; // right
    public static final Pose2d SE_left = CoralPose.getReefPose(17)[1]; // left
    public static final Pose2d S_right = CoralPose.getReefPose(18)[0]; // right
    public static final Pose2d S_left = CoralPose.getReefPose(18)[1]; // left
    public static final Pose2d SW_right = CoralPose.getReefPose(19)[0]; // right
    public static final Pose2d SW_left = CoralPose.getReefPose(19)[1]; // left
    public static final Pose2d NW_right = CoralPose.getReefPose(20)[0]; // right
    public static final Pose2d NW_left = CoralPose.getReefPose(20)[1]; // left
  }

  public static class RedReefPoses {
    public static final Pose2d N_right = CoralPose.getReefPose(10)[0]; // right
    public static final Pose2d N_left = CoralPose.getReefPose(10)[1]; // left
    public static final Pose2d NE_right = CoralPose.getReefPose(9)[0]; // right
    public static final Pose2d NE_left = CoralPose.getReefPose(9)[1]; // left
    public static final Pose2d SE_right = CoralPose.getReefPose(8)[0]; // right Thisone is so annoying
    public static final Pose2d SE_left = CoralPose.getReefPose(8)[1]; // left
    public static final Pose2d S_right = CoralPose.getReefPose(7)[0]; // right
    public static final Pose2d S_left = CoralPose.getReefPose(7)[1]; // left
    public static final Pose2d SW_right = CoralPose.getReefPose(6)[0]; // right
    public static final Pose2d SW_left = CoralPose.getReefPose(6)[1]; // left
    public static final Pose2d NW_right = CoralPose.getReefPose(11)[0]; // right
    public static final Pose2d NW_left = CoralPose.getReefPose(11)[1]; // left
  }

  public static final Pose2d Barge =
      new Pose2d(new Translation2d(7.576, 6.150), Rotation2d.fromDegrees(0));

  public static final Pose2d CageShallow =
      new Pose2d(new Translation2d(8.023, 6.146), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepLeft =
      new Pose2d(new Translation2d(8.023, 7.253), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepRight =
      new Pose2d(new Translation2d(8.023, 5.059), Rotation2d.fromDegrees(0));

  public static final Pose2d CoralStationRight =
      new Pose2d(new Translation2d(1.567, 6.579), Rotation2d.fromDegrees(125.538));

  public static final Pose2d CoralStationLeft =
      new Pose2d(new Translation2d(1.567, 1.424), Rotation2d.fromDegrees(231.639));
}

// porcesor barge feeding station
