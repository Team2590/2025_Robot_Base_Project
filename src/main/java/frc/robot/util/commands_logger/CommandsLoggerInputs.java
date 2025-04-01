package frc.robot.util.commands_logger;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CommandsLoggerInputs {
  public String[] initializingCommands = new String[0];
  public String[] executingCommands = new String[0];
  public String[] finishingCommands = new String[0];
  public String[] interruptedCommands = new String[0];
}
