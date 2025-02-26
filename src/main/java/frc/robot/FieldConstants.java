package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.util.HashMap;
import java.util.Map;

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
    public static final Pose2d Sright =
        new Pose2d(new Translation2d(3.15, 3.87), new Rotation2d(0));
    public static final Pose2d SEleft =
        new Pose2d(new Translation2d(3.62, 2.933), new Rotation2d(60));
    public static final Pose2d SEright =
        new Pose2d(new Translation2d(3.93, 2.787), new Rotation2d(60));
    public static final Pose2d NEleft =
        new Pose2d(new Translation2d(5.031, 2.787), new Rotation2d(120));
    public static final Pose2d NEright =
        new Pose2d(new Translation2d(5.304, 2.933), new Rotation2d(120));
    public static final Pose2d Nleft =
        new Pose2d(new Translation2d(5.83, 3.859), new Rotation2d(180));
    public static final Pose2d Nright =
        new Pose2d(new Translation2d(5.83, 4.17), new Rotation2d(180));
    public static final Pose2d NWleft =
        new Pose2d(new Translation2d(5.275, 5.127), new Rotation2d(-120));
    public static final Pose2d NWright =
        new Pose2d(new Translation2d(5.026, 5.24), new Rotation2d(-120));
    public static final Pose2d SWleft =
        new Pose2d(new Translation2d(3.9, 5.3), new Rotation2d(-60));
    public static final Pose2d SWright =
        new Pose2d(new Translation2d(3.676, 5), new Rotation2d(-60));
    public static final Pose2d Sleft = new Pose2d(new Translation2d(3.14, 4.2), new Rotation2d(0));
    public static final Pose2d CoralSourceRight =
        new Pose2d(new Translation2d(1.567, 6.579), Rotation2d.fromDegrees(125.538));
    public static final Pose2d CoralSourceLeft =
        new Pose2d(new Translation2d(1.567, 1.424), Rotation2d.fromDegrees(231.639));
  }

  public static class RedReefPoses {
    public static final Pose2d Sright =
        new Pose2d(new Translation2d(14.411, 4.171), new Rotation2d(180));
    public static final Pose2d Sleft =
        new Pose2d(new Translation2d(14.39, 3.869), new Rotation2d(180));
    public static final Pose2d SWright =
        new Pose2d(new Translation2d(13.923, 2.933), new Rotation2d(120));
    public static final Pose2d SWleft =
        new Pose2d(new Translation2d(13.621, 2.777), new Rotation2d(120));
    public static final Pose2d NWright =
        new Pose2d(new Translation2d(12.5, 2.806), new Rotation2d(60));
    public static final Pose2d NWleft =
        new Pose2d(new Translation2d(12.236, 2.943), new Rotation2d(60));
    public static final Pose2d Nright =
        new Pose2d(new Translation2d(11.7, 3.869), new Rotation2d(0));
    public static final Pose2d Nleft =
        new Pose2d(new Translation2d(12.217, 5.127), new Rotation2d(-60));
    public static final Pose2d NEright =
        new Pose2d(new Translation2d(11.72, 4.2), new Rotation2d(0));
    public static final Pose2d NEleft =
        new Pose2d(new Translation2d(13.913, 5.088), new Rotation2d(-120));
    public static final Pose2d SEright =
        new Pose2d(new Translation2d(13.621, 5.253), new Rotation2d(-120));
    public static final Pose2d SEleft =
        new Pose2d(new Translation2d(12.47, 5.253), new Rotation2d(-60));
    public static final Pose2d CoralSourceRight =
        new Pose2d(new Translation2d(16.005, 6.579), Rotation2d.fromDegrees(125.538));
    public static final Pose2d CoralSourceLeft =
        new Pose2d(new Translation2d(16.005, 1.424), Rotation2d.fromDegrees(231.639));
  }

  public static final Pose2d Barge =
      new Pose2d(new Translation2d(7.576, 6.150), Rotation2d.fromDegrees(0));

  public static final Pose2d CageShallow =
      new Pose2d(new Translation2d(8.023, 6.146), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepLeft =
      new Pose2d(new Translation2d(8.023, 7.253), Rotation2d.fromDegrees(0));

  public static final Pose2d CageDeepRight =
      new Pose2d(new Translation2d(8.023, 5.059), Rotation2d.fromDegrees(0));

  public static final Map<String, Pose2d> BLUE_REEF_POSES = buildBlueReefPosesMap();
  public static final Map<String, Pose2d> RED_REEF_POSES = buildRedReefPosesMap();

  private static final Map<String, Pose2d> buildBlueReefPosesMap() {
    Map<String, Pose2d> poseMap = new HashMap<>();
    poseMap.put("S_Right", FieldConstants.BlueReefPoses.Sright);
    poseMap.put("S_Left", FieldConstants.BlueReefPoses.Sleft);
    poseMap.put("SW_Right", FieldConstants.BlueReefPoses.SWright);
    poseMap.put("SW_Left", FieldConstants.BlueReefPoses.SWleft);
    poseMap.put("NW_Right", FieldConstants.BlueReefPoses.NWright);
    poseMap.put("NW_Left", FieldConstants.BlueReefPoses.NWleft);
    poseMap.put("N_Right", FieldConstants.BlueReefPoses.Nright);
    poseMap.put("N_Left", FieldConstants.BlueReefPoses.Nleft);
    poseMap.put("NE_Right", FieldConstants.BlueReefPoses.NEright);
    poseMap.put("NE_Left", FieldConstants.BlueReefPoses.NEleft);
    poseMap.put("SE_Right", FieldConstants.BlueReefPoses.SEright);
    poseMap.put("SE_Left", FieldConstants.BlueReefPoses.SEleft);
    poseMap.put("sourceL", FieldConstants.BlueReefPoses.CoralSourceLeft);
    poseMap.put("sourceR", FieldConstants.BlueReefPoses.CoralSourceRight);
    return poseMap;
  }

  private static final Map<String, Pose2d> buildRedReefPosesMap() {
    Map<String, Pose2d> poseMap = new HashMap<>();
    poseMap.put("S_Right", FieldConstants.RedReefPoses.Sright);
    poseMap.put("S_Left", FieldConstants.RedReefPoses.Sleft);
    poseMap.put("SW_Right", FieldConstants.RedReefPoses.SWright);
    poseMap.put("SW_Left", FieldConstants.RedReefPoses.SWleft);
    poseMap.put("NW_Right", FieldConstants.RedReefPoses.NWright);
    poseMap.put("NW_Left", FieldConstants.RedReefPoses.NWleft);
    poseMap.put("N_Right", FieldConstants.RedReefPoses.Nright);
    poseMap.put("N_Left", FieldConstants.RedReefPoses.Nleft);
    poseMap.put("NE_Right", FieldConstants.RedReefPoses.NEright);
    poseMap.put("NE_Left", FieldConstants.RedReefPoses.NEleft);
    poseMap.put("SE_Right", FieldConstants.RedReefPoses.SEright);
    poseMap.put("SE_Left", FieldConstants.RedReefPoses.SEleft);
    poseMap.put("sourceL", FieldConstants.RedReefPoses.CoralSourceLeft);
    poseMap.put("sourceR", FieldConstants.RedReefPoses.CoralSourceRight);
    return poseMap;
  }
}

// porcesor barge feeding station
