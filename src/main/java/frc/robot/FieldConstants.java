package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
  public static enum ZONES {
    REEF,
    BARGE,
    PROCCESOR,
    NOTE
  }

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static double fieldLength = aprilTagLayout.getFieldLength();

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
    bounds.moveTo(x[1], y[1]);

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

  private static Transform2d sourceTagToSetpoint =
      new Pose2d(
              new Translation2d(1.47, .71),
              aprilTagLayout.getTagPose(12).get().toPose2d().getRotation())
          .minus((aprilTagLayout.getTagPose(12).get().toPose2d()));

  public static class BlueReefPoses {
    static Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    static {
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();
    }

    static Pose2d[][] reef = getReefPoses(centerFaces, false);
    public static final Pose2d S_left = reef[0][0]; // left
    public static final Pose2d S_right = reef[0][1]; // right
    public static final Pose2d SW_left = reef[1][0];
    public static final Pose2d SW_right = reef[1][1];
    public static final Pose2d NW_right =
        reef[2][1]; // flip all north poses to keep robot-centri left/right
    public static final Pose2d NW_left = reef[2][0];
    public static final Pose2d N_right = reef[3][1];
    public static final Pose2d N_left = reef[3][0];
    public static final Pose2d NE_right = reef[4][1];
    public static final Pose2d NE_left = reef[4][0];
    public static final Pose2d SE_right = reef[5][1];
    public static final Pose2d SE_left = reef[5][0];
    public static final Pose2d CoralSourceRight =
        new Pose2d(
            new Translation2d(1.47, .71),
            aprilTagLayout
                .getTagPose(12)
                .get()
                .toPose2d()
                .getRotation()); // aprilTagLayout.getTagPose(12).get().toPose2d();
    public static final Pose2d CoralSourceLeft =
        new Pose2d(
            new Translation2d(1.47, 7.34),
            aprilTagLayout.getTagPose(13).get().toPose2d().getRotation());

    // aprilTagLayout
    //     .getTagPose(13)
    //     .get()
    //     .toPose2d()
    //     .transformBy(
    //         new Transform2d(
    //             new Translation2d(-sourceTagToSetpoint.getX(), sourceTagToSetpoint.getY()),
    //             new Rotation2d())); //  aprilTagLayout.getTagPose(13).get().toPose2d();
  }

  public static final Pose2d[] RedReefPosesArray =
      new Pose2d[] {
        RedReefPoses.N_right,
        RedReefPoses.N_left,
        RedReefPoses.NE_right,
        RedReefPoses.NE_left,
        RedReefPoses.SE_right,
        RedReefPoses.SE_left,
        RedReefPoses.S_right,
        RedReefPoses.S_left,
        RedReefPoses.SW_right,
        RedReefPoses.SW_left,
        RedReefPoses.NW_right,
        RedReefPoses.NW_left
      };

  public static final Pose2d[] BlueReefPosesArray =
      new Pose2d[] {
        BlueReefPoses.N_right,
        BlueReefPoses.N_left,
        BlueReefPoses.NE_right,
        BlueReefPoses.NE_left,
        BlueReefPoses.SE_right,
        BlueReefPoses.SE_left,
        BlueReefPoses.S_right,
        BlueReefPoses.S_left,
        BlueReefPoses.SW_right,
        BlueReefPoses.SW_left,
        BlueReefPoses.NW_right,
        BlueReefPoses.NW_left
      };

  public static class RedReefPoses {
    static Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    static {
      centerFaces[0] = aprilTagLayout.getTagPose(7).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(8).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(9).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(10).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(11).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(6).get().toPose2d();
    }

    static Pose2d[][] reef = getReefPoses(centerFaces, true);
    public static final Pose2d S_left = reef[3][0]; // left
    public static final Pose2d S_right = reef[3][1]; // right
    public static final Pose2d SW_left = reef[4][0];
    public static final Pose2d SW_right = reef[4][1];
    public static final Pose2d NW_right =
        reef[5][1]; // flip all north poses to keep robot-centri left/right
    public static final Pose2d NW_left = reef[5][0];
    public static final Pose2d N_right = reef[0][1];
    public static final Pose2d N_left = reef[0][0];
    public static final Pose2d NE_right = reef[1][1];
    public static final Pose2d NE_left = reef[1][0];
    public static final Pose2d SE_right = reef[2][1];
    public static final Pose2d SE_left = reef[2][0];

    public static final Pose2d CoralSourceRight =
        new Pose2d(
            new Translation2d(16.08, 7.34),
            aprilTagLayout.getTagPose(2).get().toPose2d().getRotation());
    // aprilTagLayout.getTagPose(2).get().toPose2d().transformBy(sourceTagToSetpoint);

    // aprilTagLayout.getTagPose(2).get().toPose2d();
    public static final Pose2d CoralSourceLeft =
        new Pose2d(
            new Translation2d(16.08, .71),
            aprilTagLayout.getTagPose(1).get().toPose2d().getRotation());
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
    poseMap.put("S_Right", FieldConstants.BlueReefPoses.S_right);
    poseMap.put("S_Left", FieldConstants.BlueReefPoses.S_left);
    poseMap.put("SW_Right", FieldConstants.BlueReefPoses.SW_right);
    poseMap.put("SW_Left", FieldConstants.BlueReefPoses.SW_left);
    poseMap.put("NW_Right", FieldConstants.BlueReefPoses.NW_right);
    poseMap.put("NW_Left", FieldConstants.BlueReefPoses.NW_left);
    poseMap.put("N_Right", FieldConstants.BlueReefPoses.N_right);
    poseMap.put("N_Left", FieldConstants.BlueReefPoses.N_left);
    poseMap.put("NE_Right", FieldConstants.BlueReefPoses.NE_right);
    poseMap.put("NE_Left", FieldConstants.BlueReefPoses.NE_left);
    poseMap.put("SE_Right", FieldConstants.BlueReefPoses.SE_right);
    poseMap.put("SE_Left", FieldConstants.BlueReefPoses.SE_left);

    poseMap.put(
        "sourceR",
        FieldConstants.BlueReefPoses
            .CoralSourceRight); // aprilTagLayout.getTagPose(12).get().toPose2d());

    poseMap.put("sourceL", FieldConstants.BlueReefPoses.CoralSourceLeft);
    return poseMap;
  }

  public static void logBlueReefPoses() {
    Map<String, Pose2d> poseMap = buildBlueReefPosesMap();
    for (String s : poseMap.keySet()) {

      Pose2d pose = poseMap.get(s);
      Logger.recordOutput("PoseFor" + s, pose);
      // System.out.println("Pose for " + s + " : " + pose);
    }
  }

  private static final Map<String, Pose2d> buildRedReefPosesMap() {
    Map<String, Pose2d> poseMap = new HashMap<>();

    poseMap.put("S_Right", FieldConstants.RedReefPoses.S_right);
    poseMap.put("S_Left", FieldConstants.RedReefPoses.S_left);
    poseMap.put("SW_Right", FieldConstants.RedReefPoses.SW_right);
    poseMap.put("SW_Left", FieldConstants.RedReefPoses.SW_left);
    poseMap.put("NW_Right", FieldConstants.RedReefPoses.NW_right);
    poseMap.put("NW_Left", FieldConstants.RedReefPoses.NW_left);
    poseMap.put("N_Right", FieldConstants.RedReefPoses.N_right);
    poseMap.put("N_Left", FieldConstants.RedReefPoses.N_left);
    poseMap.put("NE_Right", FieldConstants.RedReefPoses.NE_right);
    poseMap.put("NE_Left", FieldConstants.RedReefPoses.NE_left);
    poseMap.put("SE_Right", FieldConstants.RedReefPoses.SE_right);
    poseMap.put("SE_Left", FieldConstants.RedReefPoses.SE_left);

    poseMap.put("sourceL", FieldConstants.RedReefPoses.CoralSourceLeft);
    poseMap.put("sourceR", FieldConstants.RedReefPoses.CoralSourceRight);
    return poseMap;
  }

  public static Pose2d[][] getReefPoses(Pose2d[] centerFaces, boolean redAlliance) {
    Translation2d center;
    if (redAlliance) {
      double midX = (centerFaces[0].getX() + centerFaces[3].getX()) / 2.0;
      double midY = (centerFaces[0].getY() + centerFaces[3].getY()) / 2.0;
      center = new Translation2d(midX, midY);
    } else {
      center =
          new Translation2d(Units.inchesToMeters(176.746), aprilTagLayout.getFieldWidth() / 2.0);
    }

    Logger.recordOutput("Center", new Pose2d(center, new Rotation2d()));
    Pose2d[][] returnPoses = new Pose2d[6][2];

    for (int face = 0; face < 6; face++) {
      Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
      double adjustX = Units.inchesToMeters(52.738);
      double adjustY = Units.inchesToMeters(6.469);

      var rightBranchPose =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                      .getY()),
              poseDirection.getRotation().plus(new Rotation2d(Math.PI)));
      var leftBranchPose =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                      .getY()),
              poseDirection.getRotation().plus(new Rotation2d(Math.PI)));
      returnPoses[face][0] = leftBranchPose; // Assign left pose
      returnPoses[face][1] = rightBranchPose; // Assign right pose
    }
    return returnPoses;
  }

  public static Pose2d rotateAround(Pose2d originalPose, Translation2d point, Rotation2d rot) {
    return new Pose2d(
        originalPose.getTranslation().rotateAround(point, rot),
        originalPose.getRotation().rotateBy(rot));
  }
}

// porcesor barge feeding station
