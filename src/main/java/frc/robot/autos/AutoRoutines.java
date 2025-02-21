package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.command_factories.AutoFactory.NemesisAuto;
import java.util.ArrayList;

public class AutoRoutines {

  // public static LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto
  // Choices");

  // public static Command scoobydoobydoo= new AutoFactory.NemesisAuto("driveThenScoreL4",
  // "preloadl1, source cycling", new String[]{"scoreL4","intake", "scoreL1"}).getCommand();
  public static NemesisAuto driveThenScoreL4 =
      new NemesisAuto(
          "driveThenScoreL4",
          "simple1piece",
          new String[] {"dumpL1 scoreL4"},
          Constants.currentMode == Mode.SIM);

  public static ArrayList<Boolean> getTriggerBooleans(NemesisAuto auto) {
    ArrayList<Trigger> triggers = auto.getTriggers();
    ArrayList<Boolean> triggerbooleans = new ArrayList<>();
    for (Trigger t : triggers) {

      triggerbooleans.add(t.getAsBoolean());
    }
    return triggerbooleans;
  }
}
