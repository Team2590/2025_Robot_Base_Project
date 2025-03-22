package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.command_factories.AutoFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.command_factories.ScoringFactory.Level;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class NemesisAutoBuilder {
  private static HashMap<String, Command> autoRoutines = new HashMap<String, Command>();

  public enum SourceSide {
    LEFT,
    RIGHT
  }

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
