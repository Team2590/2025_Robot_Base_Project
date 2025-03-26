package frc.robot.util.commands_logger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

// spotless:off
public class CommandsLogger {
  private static CommandsLogger instance;
  private CommandsLoggerInputsAutoLogged loggerInputs = new CommandsLoggerInputsAutoLogged();
  private ArrayList<String> initializingCommands = new ArrayList<>();
  private ArrayList<String> executingCommands = new ArrayList<>();
  private ArrayList<String> finishingCommands = new ArrayList<>();
  private ArrayList<String> interruptedCommands = new ArrayList<>();

  public CommandsLogger() {
    CommandScheduler.getInstance().onCommandInitialize(command -> initializingCommands.add(command.getName()));
    CommandScheduler.getInstance().onCommandExecute(command -> executingCommands.add(command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> finishingCommands.add(command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> interruptedCommands.add(command.getName()));
  }

  private void updateInputs(CommandsLoggerInputs inputs) {
    inputs.initializingCommands = initializingCommands.toArray(String[]::new);
    inputs.executingCommands = executingCommands.toArray(String[]::new);
    inputs.finishingCommands = finishingCommands.toArray(String[]::new);
    inputs.interruptedCommands = interruptedCommands.toArray(String[]::new);
  }

  private void clearCurrent() {
    initializingCommands.clear();
    executingCommands.clear();
    finishingCommands.clear();
    interruptedCommands.clear();
  }

  public void run() {
    updateInputs(loggerInputs);
    clearCurrent();
    Logger.processInputs("Commands Logger", loggerInputs);
  }

  public static CommandsLogger getInstance() {
    if (instance == null) instance = new CommandsLogger();
    return instance;
  }
}
// spotless:on