package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.util.NemesisAutoBuilder.ReefTarget;
import frc.robot.util.NemesisAutoBuilder.SourceSide;
import java.util.HashMap;
import java.util.function.Supplier;

public class NemesisAutoBuilderPoses {
  private static HashMap<String, Supplier<Pose2d>> map = new HashMap<>();

  static {
    map.put(
        "S_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getS_left()
                : FieldConstants.BlueReefPoses.getS_left());

    map.put(
        "S_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getS_right()
                : FieldConstants.BlueReefPoses.getS_right());

    map.put(
        "SW_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getSW_left()
                : FieldConstants.BlueReefPoses.getSW_left());

    map.put(
        "SW_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getSW_right()
                : FieldConstants.BlueReefPoses.getSW_right());

    map.put(
        "NW_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getNW_right()
                : FieldConstants.BlueReefPoses.getNW_right());

    map.put(
        "NW_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getNW_left()
                : FieldConstants.BlueReefPoses.getNW_left());

    map.put(
        "N_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getN_right()
                : FieldConstants.BlueReefPoses.getN_right());

    map.put(
        "N_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getN_left()
                : FieldConstants.BlueReefPoses.getN_left());

    map.put(
        "NE_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getNE_right()
                : FieldConstants.BlueReefPoses.getNE_right());

    map.put(
        "NE_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getNE_left()
                : FieldConstants.BlueReefPoses.getNE_left());

    map.put(
        "SE_RIGHT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getSE_right()
                : FieldConstants.BlueReefPoses.getSE_right());

    map.put(
        "SE_LEFT",
        () ->
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.RedReefPoses.getSE_left()
                : FieldConstants.BlueReefPoses.getSE_left());
  }

  public static Pose2d getReefPose(ReefTarget reefTarget) {
    return map.get(reefTarget.name()).get();
  }

  public static Pose2d getNearSourcePose(SourceSide sourceSide) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red
        ? (sourceSide == SourceSide.LEFT
            ? FieldConstants.redSourceLeftIntakePose
            : FieldConstants.redSourceRightIntakePose)
        : (sourceSide == SourceSide.LEFT
            ? FieldConstants.blueSourceLeftIntakePose
            : FieldConstants.blueSourceRightIntakePose);
  }
}
