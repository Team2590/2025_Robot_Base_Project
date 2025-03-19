package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.command_factories.AutoFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class NemesisAutoBuilder {
  private static HashMap<String, Command> autoRoutines = new HashMap<String, Command>();

  public enum SourceSide {
    LEFT,
    RIGHT
  }

  public enum ReefTarget {
    S_LEFT,
    S_RIGHT,
    SW_LEFT,
    SW_RIGHT,
    NW_RIGHT,
    NW_LEFT,
    N_RIGHT,
    N_LEFT,
    NE_RIGHT,
    NE_LEFT,
    SE_RIGHT,
    SE_LEFT
  }

  public static Command generateScoringSequence(
      ReefTarget reefTarget, Level level, SourceSide sourceSide) {
    return Commands.sequence(
        AutoFactory.driveTo(NemesisAutoBuilderPoses.getPose(reefTarget)),
        ScoringFactory.score(level),
        Commands.defer(
            () -> {
              return sourceSide == SourceSide.LEFT
                  ? AutoFactory.intakeCoralAroundSourceLeft()
                  : AutoFactory.intakeCoralAroundSourceRight();
            },
            Set.of(RobotContainer.getDrive())));
  }

  public static Command generateScoringSequence(ReefTarget reefTarget, SourceSide sourceSide) {
    return generateScoringSequence(reefTarget, Level.L4, sourceSide);
  }

  public static void addRoutine(String name, Command... sequences) {
    autoRoutines.put(name, Commands.sequence(sequences));
  }

  public static void addRoutine(String name, Command routine) {
    autoRoutines.put(name, routine);
  }

  public static void addRoutine(String name, SourceSide side, Pose2d... targets) {
    ArrayList<Command> generated = new ArrayList<Command>();
    for (Pose2d target : targets) {
      generated.add(
          Commands.sequence(
              AutoFactory.driveTo(target),
              ScoringFactory.score(Level.L4),
              side == SourceSide.LEFT
                  ? AutoFactory.intakeCoralAroundSourceLeft()
                  : AutoFactory.intakeCoralAroundSourceRight()));
    }
    autoRoutines.put(name, Commands.sequence(generated.toArray(Command[]::new)));
  }

  public static SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    autoRoutines.forEach((name, routine) -> chooser.addOption(name, routine));
    return chooser;
  }

  public static void addRoutinesToChooser(LoggedDashboardChooser<Command> chooser) {
    autoRoutines.forEach((name, routine) -> chooser.addOption(name, routine));
  }
}
