package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class NemesisTimedCommand {
  /**
   * Generates a command to run for a certain amount of time
   *
   * @param c - The command to be run
   * @param time - time (seconds) to run the command for
   * @return a ParallelRaceGroup with the given command and a WaitCommand of the given time
   */
  public static Command generateTimedCommand(Command c, double time) {
    return new ParallelRaceGroup(c, new WaitCommand(time));
  }
}
