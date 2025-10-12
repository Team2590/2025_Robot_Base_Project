package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
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

    static Pose2d[][] reef = getReefPoses(false);

    public static Pose2d getS_left() {
      return reef[0][0];
    } // left

    public static Pose2d getS_right() {
      return reef[0][1];
    } // right

    public static Pose2d getSW_left() {
      return reef[1][0];
    }

    public static Pose2d getSW_right() {
      return reef[1][1];
    }

    public static Pose2d getNW_right() {
      return reef[2][1];
    } // flip all north poses to keep robot-centri left/right

    public static Pose2d getNW_left() {
      return reef[2][0];
    }

    public static Pose2d getN_right() {
      return reef[3][1];
    }

    public static Pose2d getN_left() {
      return reef[3][0];
    }

    public static Pose2d getNE_right() {
      return reef[4][1];
    }

    public static Pose2d getNE_left() {
      return reef[4][0];
    }

    public static Pose2d getSE_right() {
      return reef[5][1];
    }

    public static Pose2d getSE_left() {
      return reef[5][0];
    }

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
  }

  public static class RedReefPoses {
    public static Pose2d[][] reef = getReefPoses(true);

    public static Pose2d getS_left() {
      return reef[3][0];
    } // left

    public static Pose2d getS_right() {
      return reef[3][1];
    } // right

    public static Pose2d getSW_left() {
      return reef[4][0];
    }

    public static Pose2d getSW_right() {
      return reef[4][1];
    }

    public static Pose2d getNW_right() {
      return reef[5][1];
    } // flip all north poses to keep robot-centri left/right

    public static Pose2d getNW_left() {
      return reef[5][0];
    }

    public static Pose2d getN_right() {
      return reef[0][1];
    }

    public static Pose2d getN_left() {
      return reef[0][0];
    }

    public static Pose2d getNE_right() {
      return reef[1][1];
    }

    public static Pose2d getNE_left() {
      return reef[1][0];
    }

    public static Pose2d getSE_right() {
      return reef[2][1];
    }

    public static Pose2d getSE_left() {
      return reef[2][0];
    }

    public static final Pose2d CoralSourceRight =
        new Pose2d(
            new Translation2d(16.08, 7.34),
            aprilTagLayout.getTagPose(2).get().toPose2d().getRotation());
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

  public static final double BACK_SCORING_DIFFERENCE = Units.inchesToMeters(2); // .75

  public static Pose2d convertBackScoring(Pose2d frontPose) {
    double moveLeftDistanceMeters = BACK_SCORING_DIFFERENCE;
    Translation2d translation =
        new Translation2d(
            moveLeftDistanceMeters, frontPose.getRotation().plus(Rotation2d.fromDegrees(90)));
    Pose2d backPose =
        new Pose2d(
            frontPose.getTranslation().plus(translation),
            frontPose.getRotation().plus(new Rotation2d(0)));
    return backPose;
  }

  public static Map<String, Pose2d> BLUE_REEF_POSES = buildBlueReefPosesMap();
  public static Map<String, Pose2d> RED_REEF_POSES = buildRedReefPosesMap();

  private static Map<String, Pose2d> buildBlueReefPosesMap() {
    Map<String, Pose2d> poseMap = new HashMap<>();
    poseMap.put("S_Right", FieldConstants.BlueReefPoses.getS_right());
    poseMap.put("S_Left", FieldConstants.BlueReefPoses.getS_left());
    poseMap.put("SW_Right", FieldConstants.BlueReefPoses.getSW_right());
    poseMap.put("SW_Left", FieldConstants.BlueReefPoses.getSW_left());
    poseMap.put("NW_Right", FieldConstants.BlueReefPoses.getNW_right());
    poseMap.put("NW_Left", FieldConstants.BlueReefPoses.getNW_left());
    poseMap.put("N_Right", FieldConstants.BlueReefPoses.getN_right());
    poseMap.put("N_Left", FieldConstants.BlueReefPoses.getN_left());
    poseMap.put("NE_Right", FieldConstants.BlueReefPoses.getNE_right());
    poseMap.put("NE_Left", FieldConstants.BlueReefPoses.getNE_left());
    poseMap.put("SE_Right", FieldConstants.BlueReefPoses.getSE_right());
    poseMap.put("SE_Left", FieldConstants.BlueReefPoses.getSE_left());

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
    }
  }

  private static Map<String, Pose2d> buildRedReefPosesMap() {
    Map<String, Pose2d> poseMap = new HashMap<>();
    poseMap.put("S_Right", FieldConstants.RedReefPoses.getS_right());
    poseMap.put("S_Left", FieldConstants.RedReefPoses.getS_left());
    poseMap.put("SW_Right", FieldConstants.RedReefPoses.getSW_right());
    poseMap.put("SW_Left", FieldConstants.RedReefPoses.getSW_left());
    poseMap.put("NW_Right", FieldConstants.RedReefPoses.getNW_right());
    poseMap.put("NW_Left", FieldConstants.RedReefPoses.getNW_left());
    poseMap.put("N_Right", FieldConstants.RedReefPoses.getN_right());
    poseMap.put("N_Left", FieldConstants.RedReefPoses.getN_left());
    poseMap.put("NE_Right", FieldConstants.RedReefPoses.getNE_right());
    poseMap.put("NE_Left", FieldConstants.RedReefPoses.getNE_left());
    poseMap.put("SE_Right", FieldConstants.RedReefPoses.getSE_right());
    poseMap.put("SE_Left", FieldConstants.RedReefPoses.getSE_left());

    poseMap.put("sourceL", FieldConstants.RedReefPoses.CoralSourceLeft);
    poseMap.put("sourceR", FieldConstants.RedReefPoses.CoralSourceRight);
    return poseMap;
  }

  public static Pose2d[][] getReefPoses(boolean redAlliance) {
    Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in clockwise order
    if (redAlliance) {
      centerFaces[0] = aprilTagLayout.getTagPose(7).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(8).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(9).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(10).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(11).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(6).get().toPose2d();
    } else {
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();
    }
    Translation2d center;
    if (redAlliance) {
      double midX = (centerFaces[0].getX() + centerFaces[3].getX()) / 2.0;
      double midY = (centerFaces[0].getY() + centerFaces[3].getY()) / 2.0;
      center = new Translation2d(midX, midY);
    } else {
      center =
          new Translation2d(Units.inchesToMeters(176.746), aprilTagLayout.getFieldWidth() / 2.0);
    }

    Pose2d[][] returnPoses = new Pose2d[6][2];

    for (int face = 0; face < 6; face++) {
      Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
      double adjustY = Units.inchesToMeters(52.738 + Drive.reefYOffset.get());
      double adjustXLeft =
          Units.inchesToMeters(7.87); // + RobotState.getInstance().getReefOffsetLeft());
      double adjustXRight =
          Units.inchesToMeters(7.87 - 2.5); // + RobotState.getInstance().getReefOffsetRight());

      // System.out.println("updating offsets to " + adjustY);

      var rightBranchPose =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(new Transform2d(adjustY, adjustXRight, Rotation2d.kZero))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustY, adjustXRight, Rotation2d.kZero))
                      .getY()),
              poseDirection.getRotation().plus(new Rotation2d(Math.PI)));
      var leftBranchPose =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(new Transform2d(adjustY, -adjustXLeft, Rotation2d.kZero))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustY, -adjustXLeft, Rotation2d.kZero))
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

  public static void updateTunableNumbers() {
    // logBlueReefPoses();
    // Rebuild the reef poses if y offset is changed in AdvantageScope
    if (Drive.reefYOffset.hasChanged("ReefYOffset".hashCode())) {
      // System.out.println("Rebuilding reef poses, new y offset: " + Drive.reefYOffset.get());
      BlueReefPoses.reef = getReefPoses(false);
      RedReefPoses.reef = getReefPoses(true);
      BLUE_REEF_POSES = buildBlueReefPosesMap();
      RED_REEF_POSES = buildRedReefPosesMap();
    }

    logBlueReefPoses();
  }
}

// porcesor barge feeding station
