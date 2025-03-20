package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command_factories.AutoFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import frc.robot.util.NemesisAutoBuilder.SourceSide;
import java.util.HashMap;
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
        AutoFactory.driveTo(NemesisAutoBuilderPoses.getReefPose(reefTarget)),
        ScoringFactory.score(level),
        AutoFactory.driveTo(NemesisAutoBuilderPoses.getNearSourcePose(sourceSide)));
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

  public static SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    autoRoutines.forEach((name, routine) -> chooser.addOption(name, routine));
    return chooser;
  }

  public static void addRoutinesToChooser(LoggedDashboardChooser<Command> chooser) {
    autoRoutines.forEach((name, routine) -> chooser.addOption(name, routine));
  }
}
