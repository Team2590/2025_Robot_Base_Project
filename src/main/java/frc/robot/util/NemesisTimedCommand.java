package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NemesisTimedCommand {
  /**
   * Generates a command to run for a certain amount of time
   *
   * @param c
   * @param time
   * @return
   */
  public static Command generateTimedCommand(Command c, double time) {
    return new ParallelRaceGroup(c, new WaitCommand(time));
  }
}
