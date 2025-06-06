package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState.ReefTargetSide;
import frc.robot.command_factories.ArmFactory;
import frc.robot.command_factories.GamePieceFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NemesisDriveToPoseStraight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

public class ControllerOrchestrator {

  private static final String CONTROLLER_TABLE_KEY = "ControllerApp/target";
  private static final String MOVE_TO_KEY = "moveTo";
  private static final String SOURCE_KEY = "source";
  // What's a better default target location?
  private static final String DEFAULT_REEF_TARGET = "S_Left";
  private static final String DEFAULT_SOURCE_TARGET = "sourceR";

  // Compass directions in clockwise order - used to find the closest direction
  private static final String[] COMPASS_DIRECTIONS = {"N", "NE", "SE", "S", "SW", "NW"};

  private Target target = new Target(lookupPoseBasedOnAlliance("SW_Left"), Level.L4);

  private NetworkTableEntry getTableEntry(String key) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(CONTROLLER_TABLE_KEY);
    return table.getEntry(key);
  }

  private String getValue(String key) {
    try {
      return getTableEntry(key).getString("<No value set in NetworkTable>");
    } catch (Exception e) {
      return "";
    }
  }

  public String getMoveTo() {
    return getValue(MOVE_TO_KEY);
  }

  public String getSource() {
    return getValue(SOURCE_KEY);
  }

  public Target getTarget() {
    // return target;
    String side = cachedReefTargetSide == ReefTargetSide.LEFT ? "Left" : "Right";
    return new Target(
        lookupPoseBasedOnAlliance(determineCompassDirection() + "_" + side), cachedLevel);
    // Target target = parseTargetString(getMoveTo());
    // if (target == null) {
    //   target = new Target(lookupPoseBasedOnAlliance(DEFAULT_REEF_TARGET),
    // ScoringFactory.Level.L4);
    //   // System.err.println("---> Using Default Target: " + target);
    //   return target;
    // }
    // return target;
  }

  public String getReefSide() {
    return cachedReefTargetSide.name();
  }

  public Target getSourceTarget() {
    Target target;
    Pose2d pose = lookupPoseBasedOnAlliance(getSource());
    if (pose == null) {
      // System.err.println("---> Using Default Source Target: ");
      return new Target(
          lookupPoseBasedOnAlliance(DEFAULT_SOURCE_TARGET), ScoringFactory.Level.SOURCE);
    }
    return new Target(lookupPoseBasedOnAlliance(getSource()), ScoringFactory.Level.SOURCE);
  }

  /**
   * Command that needs to be bound to a button to execute scoring at the level specified by
   * Controller App.
   */
  public Command bindScoringCommand(Elevator elevator, Arm arm) {
    // var requirements = new HashSet<Subsystem>();
    // // requirements.add(elevator);
    // requirements.add(arm);

    // return Commands.defer(
    //     () -> {
    //       System.out.println(
    //           "arm place target: "
    //               + RobotState.getInstance().getCoralScoringSetpoints().armPlaceSetpoint);
    return ArmFactory.setPositionBlocking(.015);
    // ScoringFactory.score(RobotState.getInstance().getCoralScoringSetpoints());
    // },
    // requirements);
  }

  /** Command that needs to be bound to a button to driveToTarget. */
  public Command bindDriveToTargetCommand(Drive drive) {
    return new NemesisDriveToPoseStraight(drive, () -> RobotState.getInstance().getTargetPose());
  }

  /** Command that needs to be bound to a button to driveToTarget. */
  public Command driveAndAutoScoreCommand(Drive drive, Elevator elevator, Arm arm) {
    var requirements = new HashSet<Subsystem>();
    requirements.add(elevator);
    requirements.add(arm);
    return Commands.defer(
        () -> {
          return Commands.parallel(
              new NemesisDriveToPoseStraight(drive, () -> RobotState.getInstance().getTargetPose()),
              ScoringFactory.primeForLevel(RobotState.getInstance().getCoralScoringSetpoints()));
        },
        requirements);
  }

  // This commands will drive to pose while "priming for intake" at coral source
  public Command bindDriveToSourceIntake(Drive drive) {
    var requirements = new HashSet<Subsystem>();
    requirements.add(drive);
    return Commands.defer(
        () -> {
          return new ParallelCommandGroup(
              DriveCommands.preciseAlignmentAutoBuilder(
                  drive, () -> getSourceTarget().pose(), getSourceTarget().pose().getRotation()),
              GamePieceFactory.intakeCoralGroundAndHandoff());
        },
        requirements);
  }

  private Target parseTargetString(String targetString) {
    // The new format from controller app is now: Side_Level
    // where Side is Left/Right, Level is L2/L3/L4
    String[] parts = targetString.split("_");
    if (parts.length != 2) {
      // System.err.println("---> Invalid target string received from ControllerApp: " +
      // targetString);
      return null;
    }

    String leftOrRight = parts[0];
    String levelString = parts[1].toUpperCase();

    // Find the closest compass direction based on the robot's current position
    String compassDir = determineCompassDirection();

    // Construct the pose key using the determined compass direction and the side from controller
    String poseKey = compassDir + "_" + leftOrRight;

    Pose2d targetPose = lookupPoseBasedOnAlliance(poseKey);
    if (targetPose == null) {
      // System.err.println(
      //     "---> Caution!!! Invalid target pose key received from ControllerApp: " + poseKey);
      return null;
    }

    ScoringFactory.Level level = ScoringFactory.Level.valueOf(levelString);
    return new Target(targetPose, level);
  }

  private Level cachedLevel = Level.L4;
  private ReefTargetSide cachedReefTargetSide = ReefTargetSide.LEFT;

  public void setTarget(Level level) {
    cachedLevel = level;
  }

  public void setTarget(ReefTargetSide side) {
    System.out.println("Set reef target side called");
    cachedReefTargetSide = side;
  }

  /**
   * Determines the closest compass direction based on the robot's current position. This method
   * analyzes where the robot is on the field and returns the most appropriate compass direction (N,
   * NE, SE, S, SW, NW) for targeting.
   *
   * @return The compass direction as a String
   */
  public String determineCompassDirection() {
    // Get the robot's current position
    Pose2d currentPose = RobotState.getInstance().getPose();

    // If we can't determine the current pose, default to "N"
    if (currentPose == null) {
      return "N";
    }

    // Get alliance-specific field positions to determine which side of the field we're on
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Alliance allianceValue = alliance.orElse(Alliance.Red);

    // Create a map to track possible distances to each compass location
    Map<Double, String> directionDistances = new TreeMap<>(); // TreeMap to keep it sorted

    // For each compass direction, calculate a distance metric from the robot's position
    // to determine which reef position would be most appropriate
    for (String direction : COMPASS_DIRECTIONS) {
      // For both Left and Right sides (we'll pick the closest one)
      for (String side : new String[] {"Left", "Right"}) {
        String poseKey = direction + "_" + side;
        Pose2d targetPose = lookupPoseBasedOnAlliance(poseKey);

        if (targetPose != null) {
          // Calculate distance (simple Euclidean distance for now)
          double distance = calculateDistance(currentPose, targetPose);
          directionDistances.put(distance, direction);
        }
      }
    }

    // Return the closest direction (first entry in the sorted map)
    // If no valid directions found, default to "N"
    return directionDistances.isEmpty() ? "N" : directionDistances.values().iterator().next();
  }

  /** Calculate distance between two poses (simple Euclidean distance) */
  private double calculateDistance(Pose2d pose1, Pose2d pose2) {
    double dx = pose1.getX() - pose2.getX();
    double dy = pose1.getY() - pose2.getY();
    return Math.sqrt(dx * dx + dy * dy);
  }

  private static final Pose2d lookupPoseBasedOnAlliance(String poseKey) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Alliance alianceValue;
    if (alliance.isPresent()) {
      alianceValue = alliance.get();
    } else {
      // System.err.println("---> Caution!!! Alliance not found, defaulting to Red");
      alianceValue = Alliance.Red;
    }
    return alianceValue == Alliance.Blue
        ? FieldConstants.BLUE_REEF_POSES.get(poseKey)
        : FieldConstants.RED_REEF_POSES.get(poseKey);
  }

  public record Target(Pose2d pose, ScoringFactory.Level scoringLevel) {}
}
