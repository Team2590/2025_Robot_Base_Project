package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashSet;
import java.util.Optional;

public class ControllerOrchestrator {

  private static final String CONTROLLER_TABLE_KEY = "ControllerApp/target";
  private static final String MOVE_TO_KEY = "moveTo";
  // What's a better default target location?
  private static final String DEFAULT_REEF_TARGET = "Sleft";

  private NetworkTableEntry getTableEntry(String key) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(CONTROLLER_TABLE_KEY);
    return table.getEntry(key);
  }

  private String getValue(String key) {
    return getTableEntry(key).getString("<No value set in NetworkTable>");
  }

  public String getMoveTo() {
    return getValue(MOVE_TO_KEY);
  }

  public Target getTarget() {
    Target target = parseTargetString(getMoveTo());
    if (target == null) {
      System.err.println("---> Using Default Target: " + getMoveTo());
      return new Target(lookupPoseBasedOnAlliance(DEFAULT_REEF_TARGET), ScoringFactory.Level.L4);
    }
    return target;
  }

  /**
   * Command that needs to be bound to a button to execute scoring at the level specified by
   * Controller App.
   */
  public Command bindScoringCommand(Elevator elevator, Arm arm) {
    var requirements = new HashSet<Subsystem>();
    requirements.add(elevator);
    requirements.add(arm);

    return Commands.defer(
        () -> {
          Target target = getTarget();
          return ScoringFactory.score(target.scoringLevel());
        },
        requirements);
  }

  /** Command that needs to be bound to a button to driveToTarget. */
  public Command bindDriveToTargetCommand(Drive drive) {
    return DriveCommands.driveToPose(drive, () -> getTarget().pose());
  }

  private static Target parseTargetString(String targetString) {
    // We expect the string to be in the form NWright_L1
    // If we get something else, we return null so a default
    // value can be used.
    String[] parts = targetString.split("_");
    if (parts.length != 2) {
      System.out.println("---> Invalid target string received from ControllerApp: " + targetString);
      return null;
    }
    String poseKey = parts[0];
    String levelString = parts[1].toUpperCase();

    Pose2d targetPose = lookupPoseBasedOnAlliance(poseKey);
    if (targetPose == null) {
      System.err.println(
          "---> Caution!!! Invalid target pose key received from ControllerApp: " + poseKey);
      return null;
    }
    ScoringFactory.Level level = ScoringFactory.Level.valueOf(levelString);
    return new Target(targetPose, level);
  }

  private static final Pose2d lookupPoseBasedOnAlliance(String poseKey) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Alliance alianceValue;
    if (alliance.isPresent()) {
      alianceValue = alliance.get();
    } else {
      System.err.println("---> Caution!!! Alliance not found, defaulting to Red");
      alianceValue = Alliance.Red;
    }
    return alianceValue == Alliance.Blue
        ? FieldConstants.BLUE_REEF_POSES.get(poseKey)
        : FieldConstants.RED_REEF_POSES.get(poseKey);
  }

  public record Target(Pose2d pose, ScoringFactory.Level scoringLevel) {}
}
