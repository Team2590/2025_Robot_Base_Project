package frc.robot;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

  public static class BlueCoralPoses {
    public static final Pose2d p1 = new Pose2d(new Translation2d(3.15, 3.87), new Rotation2d(0));
    public static final Pose2d p2 = new Pose2d(new Translation2d(3.62, 2.933), new Rotation2d(60));
    public static final Pose2d p3 = new Pose2d(new Translation2d(3.93, 2.787), new Rotation2d(60));
    public static final Pose2d p4 = new Pose2d(new Translation2d(5.031,2.787), new Rotation2d(120));
    public static final Pose2d p5 = new Pose2d(new Translation2d(5.304,2.933), new Rotation2d(120));
    public static final Pose2d p6 = new Pose2d(new Translation2d(5.83, 3.859), new Rotation2d(180));
    public static final Pose2d p7 = new Pose2d(new Translation2d(5.83, 4.17), new Rotation2d(180));
    public static final Pose2d p8 = new Pose2d(new Translation2d(5.275, 5.127), new Rotation2d(-120));
    public static final Pose2d p9 = new Pose2d(new Translation2d(5.026, 5.24), new Rotation2d(-120));
    public static final Pose2d p10 = new Pose2d(new Translation2d(3.9, 5.3), new Rotation2d(-60));
    public static final Pose2d p11 = new Pose2d(new Translation2d(3.676, 5), new Rotation2d(-60));
    public static final Pose2d p12 = new Pose2d(new Translation2d(3.14, 4.2), new Rotation2d(0));
  }

  public static class RedCoralPoses {    
    public static final Pose2d p1 = new Pose2d(new Translation2d(14.411, 4.171), new Rotation2d(180));
    public static final Pose2d p2 = new Pose2d(new Translation2d(14.39, 3.869), new Rotation2d(180));
    public static final Pose2d p3 = new Pose2d(new Translation2d(13.923, 2.933), new Rotation2d(120));
    public static final Pose2d p4 = new Pose2d(new Translation2d(13.621, 2.777), new Rotation2d(120));
    public static final Pose2d p5 = new Pose2d(new Translation2d(12.5, 2.806), new Rotation2d(60));
    public static final Pose2d p6 = new Pose2d(new Translation2d(12.236, 2.943), new Rotation2d(60));
    public static final Pose2d p7 = new Pose2d(new Translation2d(11.7, 3.869), new Rotation2d(0));
    public static final Pose2d p9 = new Pose2d(new Translation2d(12.217, 5.127), new Rotation2d(-60));
    public static final Pose2d p8 = new Pose2d(new Translation2d(11.72, 4.2), new Rotation2d(0));
    public static final Pose2d p12 = new Pose2d(new Translation2d(13.913, 5.088), new Rotation2d(-120));
    public static final Pose2d p11 = new Pose2d(new Translation2d(13.621, 5.253), new Rotation2d(-120));
    public static final Pose2d p10 = new Pose2d(new Translation2d(12.47, 5.253), new Rotation2d(-60));
  }
}

// porcesor barge feeding station
