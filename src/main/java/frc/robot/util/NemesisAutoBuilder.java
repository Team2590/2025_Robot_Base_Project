package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.DriveCommands;

public class NemesisAutoBuilder {

  public enum ReefTarget {
    S_Right,
    S_Left,
    SW_Right,
    SW_Left,
    NW_Right,
    NW_Left,
    N_Right,
    N_Left,
    NE_Right,
    NE_Left,
    SE_Right,
    SE_Left
  }

  private static Command driveAndScore(ReefTarget reefTarget, Level level) {
    String name = reefTarget + "_" + level;
    Command command =
        Commands.sequence(driveToPoseCommandForAuto(reefTarget), ScoringFactory.score(level))
            .withName(name);
    NamedCommands.registerCommand(name, command);
    return command;
  }

  private static Command driveAndScoreL4(ReefTarget reefTarget) {
    return driveAndScore(reefTarget, Level.L4);
  }

  private static Pose2d getReefPose(ReefTarget reefTarget) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? FieldConstants.BLUE_REEF_POSES.get(reefTarget.name())
        : FieldConstants.RED_REEF_POSES.get(reefTarget.name());
  }

  public static void registerNamedCommandsForAutos() {
    for (ReefTarget reefTarget : ReefTarget.values()) {
      driveAndScoreL4(reefTarget);
    }
  }

  private static Command driveToPoseCommandForAuto(ReefTarget reefTarget) {
    return DriveCommands.driveToPoseStraight(
        RobotContainer.getDrive(), () -> getReefPose(reefTarget));
  }
}
