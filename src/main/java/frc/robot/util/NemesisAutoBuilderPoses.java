package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.util.NemesisAutoBuilder.ReefTarget;
import frc.robot.util.NemesisAutoBuilder.SourceSide;

public class NemesisAutoBuilderPoses {
  public static Pose2d getReefPose(ReefTarget reefTarget) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? FieldConstants.BLUE_REEF_POSES.get(reefTarget.name())
        : FieldConstants.RED_REEF_POSES.get(reefTarget.name());
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
