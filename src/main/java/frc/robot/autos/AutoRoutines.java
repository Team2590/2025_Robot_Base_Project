package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;

public class AutoRoutines {

  public static NemesisAuto driveThenScoreL4 =
      new NemesisAuto("driveThenScoreL4", "simple1piece", new String[] {"dumpL1 scoreL4"}, false);

  public static ArrayList<Boolean> getTriggerBooleans(NemesisAuto auto) {
    ArrayList<Trigger> triggers = auto.getTriggers();
    ArrayList<Boolean> triggerbooleans = new ArrayList<>();
    for (Trigger t : triggers) {

      triggerbooleans.add(t.getAsBoolean());
    }
    return triggerbooleans;
  }
}
