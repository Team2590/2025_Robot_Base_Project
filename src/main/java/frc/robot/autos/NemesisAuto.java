package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.command_factories.EndEffectorFactory;
import frc.robot.command_factories.GamePieceFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.ScoringFactory;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class NemesisAuto {

  public String fileName;
  String[] commandString;
  public PathPlannerAuto currentCommand;
  public String autoName;
  public PathPlannerAuto initialCommand;
  public boolean manualMode;
  public Command chosenCommand;
  // String chosenCommandName;
  public ArrayList<Trigger> triggers;
  List<PathPlannerPath> pathList;
  HashMap<String, Command> stringToCommand;
  public int pathI;

  // public Command eventCommand;
  // Load in auto path Group from file

  /*
   * This constructor builds an auto that creates an EventScheduler and for each Trajectory it creates a command for that trajectory
   * So command1 will be for trajectory 1, command2 will be for trajectory2, command3 will be for trajectory 3 etc
   * When we run them all in parallel, hopefully when we are on trajectory1 command1 will run along with the pathfollowcommand
   * We define triggers for certain conditions such as eventTriggers ,or custom conditions
   *
   *
   */

  public NemesisAuto(String autoName, String fileName, String[] commandString) {
    // this.manualMode = manual;
    this.fileName = fileName;
    this.commandString = commandString;
    this.autoName = autoName;
    this.pathI = 0;
    // this.chosenCommand=Commands.none();

    stringToCommand = new HashMap<>();
    stringToCommand.put(
        "scoreL4",
        (Constants.currentMode == Constants.simMode
            ? Commands.runOnce(() -> logAutoCommandsSim("Score L4 placeholder"))
            : ScoringFactory.scoreL4()));
    stringToCommand.put("scoreL3", ScoringFactory.scoreL3());
    stringToCommand.put("scoreL2", ScoringFactory.scoreL3());
    stringToCommand.put("dumpL1", ScoringFactory.scoreL1());
    stringToCommand.put("intake", GamePieceFactory.intakeCoralFeeder());
    stringToCommand.put("init", new WaitCommand(0.02));
    stringToCommand.put("scoreProcessor", ScoringFactory.scoreProcessor());

    // simMode = simOrNah;
    logAutoCommandsSim("init this auto :D : " + fileName);

    triggers = new ArrayList<>();
    try {
      pathList = PathPlannerAuto.getPathGroupFromAutoFile(fileName);
    } catch (Exception e) {
      pathList = null;
      System.out.println("oopsei with reading the . auto file " + e.getMessage());
    }
  }

  public Command getCommandStitched() {

    Command x = Commands.print("init message");
    for (int i = 0; i < pathList.size(); i++) {
      pathI = i;
      chosenCommand = stringToCommand.get(commandString[i]);
      Command followUp =
          commandString[i].contains("score")
              ? EndEffectorFactory.runEndEffectorOuttake().andThen(ScoringFactory.stow())
              : ScoringFactory.stow();
      x = x.andThen(customFollowPath.andThen(followUp));
    }
    PathPlannerAuto auto = new PathPlannerAuto(x);
    createTriggers(auto);

    return auto;
  }

  public void createTriggers(PathPlannerAuto command) {

    for (int i = 0; i < pathList.size(); i++) {
      Trigger whichPath;
      if (i == 0) {
        whichPath = command.activePath(pathList.get(0).name);
        whichPath.onTrue(Commands.runOnce(() -> setCurrentCommand("init")));
      } else {
        whichPath = command.activePath(pathList.get(i).name);
        AtomicReference<Integer> index = new AtomicReference<Integer>(i);
        whichPath.onTrue(Commands.runOnce(() -> setCurrentCommand(commandString[index.get()])));
      }

      triggers.add(whichPath);
    }
    // trigger for each path

    Trigger scoreL4Trigger = command.event("scoreL4");

    Trigger scoreL3Trigger = command.event("scoreL3");

    Trigger scoreL2Trigger = command.event("scoreL2");
    // .and(whichPath); // if in the reef and we following the same path as the selected i
    Trigger atIntakeStation =
        command.condition(
            () ->
                Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).contains("Feeder"));
    // .and(whichPath); // if in the intake zone
    Trigger scoreProcessor =
        command.event("PrimeProcessor"); // .and(whichPath); // controlled by event markers

    Trigger dumpL1 = command.event("dumpL1");
    // .and(whichPath); // if we are at EventMarker in path which has the name "dumpL1"

    Trigger intakeStationTrigger = command.event("IntakeStation");
    // whichPath.onTrue(command);
    scoreProcessor.onTrue(ScoringFactory.scoreProcessor());
    scoreL4Trigger.onTrue(ScoringFactory.scoreL4());
    scoreL3Trigger.onTrue(ScoringFactory.scoreL3());
    scoreL2Trigger.onTrue(ScoringFactory.scoreL2());
    dumpL1.onTrue(ScoringFactory.scoreL1());
    intakeStationTrigger.onTrue(GamePieceFactory.intakeCoralFeeder());

    // triggers.add(whichPath);
    // triggers.add(scoreProcessor);
    triggers.add(scoreL4Trigger);
    // triggers.add(scoreL3Trigger);
    // triggers.add(scoreL2Trigger);
    // triggers.add(dumpL1);
    // triggers.add(intakeStationTrigger);
  }

  public Command getCommand() {
    // initialize the conditionals to add Commands based on the strings in the list
    /* Ie ['scoreL4', 'intake', 'scoreL1', 'dumpL1 intake']
    first iteration
    scoreL4
    Create Triggers inReef, and atIntakeStation
    This way we create a Trigger for each iteration, and only the ones that are on the correct iteration can be run
    Activate Trigger based on string

     *
     *
     */

    Command init = AutoBuilder.buildAuto(fileName);
    currentCommand = new PathPlannerAuto(init);
    for (int i = 0; i < pathList.size(); i++) {
      Trigger whichPath =
          currentCommand.activePath(
              pathList.get(i)
                  .name); // creates a trigger thats true when we are on the currentPath of index
      // i

      // if the first path is being run, we want to use the first command in the command list
      // we are on the first path and the first command in the list, because if we want to do
      // something different while in the reef next time,
      Trigger startScoreTrigger =
          // currentCommand
          //     .condition(
          //         () ->
          //
          // Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef"))

          currentCommand
              .event("startScore")
              .and(whichPath); // if in the reef and we following the same path as the selected i
      Trigger atIntakeStation =
          currentCommand
              .condition(
                  () ->
                      Constants.locator
                          .getZoneOfField(AutoBuilder.getCurrentPose())
                          .contains("Feeder"))
              .and(whichPath); // if in the intake zone
      Trigger scoreProcessor =
          currentCommand.event("PrimeProcessor").and(whichPath); // controlled by event markers

      Trigger dumpL1 =
          currentCommand
              .event("dumpL1")
              .and(whichPath); // if we are at EventMarker in path which has the name "dumpL1"
      // dumpL1.onTrue(ScoringFactory.scoreL1());

      Command chosen;
      // ['dumpl1 score'] will activate the Trigger for dumpL1 and intake based on their
      // conditions, this way on one path we can do multiple things
      String[] currentString = commandString[i].split(" ");

      for (String command : currentString) {

        switch (command) {
            // activate Triggers !
          default:
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.print("default commnad")
                    : Commands.print("mispelled or smth").andThen(ScoringFactory.stow());
            System.out.println("default switch case");

            break;
          case "none":
            chosen = Commands.none();
            break;
          case "scoreL4":
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.runOnce(
                        () -> logAutoCommandsSim("Score L4 sim")) // on True this command should be
                    // scheduled
                    : ScoringFactory.scoreL4().andThen(ScoringFactory.stow());
            startScoreTrigger.onTrue(chosen);
            System.out.println(" \n  l4 switch case, Trigger created     \n");
            break;
          case "scoreL2":
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.runOnce(() -> logAutoCommandsSim("Score L2 sim"))
                    : ScoringFactory.scoreL1().andThen(ScoringFactory.stow());

            startScoreTrigger.onTrue(chosen);

            break;
          case "dumpL1":
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.runOnce(() -> logAutoCommandsSim("dump L1 sim"))
                        .alongWith(Commands.print("dumpL1 Command run"))
                    : ScoringFactory.scoreL1();
            dumpL1.onTrue(chosen);

          case "intakeFromStation":
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.runOnce(() -> logAutoCommandsSim("intake station sim"))
                    : EndEffectorFactory.runEndEffector().andThen(ScoringFactory.stow());

            atIntakeStation.onTrue(chosen);

            break;

          case "processor":
            chosen =
                Constants.currentMode == Constants.simMode
                    ? Commands.runOnce(() -> logAutoCommandsSim("processor score sim"))
                    : IntakeFactory.setHoldingAlgaePosition();
            scoreProcessor.onTrue(chosen);
        }

        triggers.add(startScoreTrigger);
        triggers.add(whichPath);
        triggers.add(atIntakeStation);
        triggers.add(scoreProcessor);
        triggers.add(dumpL1);
      }
    }

    return currentCommand;
  }

  private Command customFollowPath =
      new Command() {
        PathPlannerPath path;
        Command pathCommand;

        public void initialize() {

          path = pathList.get(pathI);

          pathCommand = AutoBuilder.followPath(path);
        }

        public void execute() {
          boolean shouldRun = chosenCommand.isFinished();
          if (shouldRun) {

            pathCommand.execute();
          }
        }
        ;

        public boolean isFinished() {
          return pathCommand.isFinished();
        }

        public void end(boolean interrupted) {}
        ;
      };

  public void setCurrentCommand(String s) {
    chosenCommand = stringToCommand.get(s);
  }

  public ArrayList<Trigger> getTriggers() {
    return triggers;
  }

  public static void logAutoCommandsSim(String message) {
    Drive.autoCommandMessage = message;
  }

  public static PathPlannerTrajectory getTrajectoryFromAuto(String fileName, int index) {
    RobotConfig config = RobotContainer.getDrive().getConfig();
    try {
      PathPlannerPath p = PathPlannerAuto.getPathGroupFromAutoFile(fileName).get(index);
      return p.generateTrajectory(
          new ChassisSpeeds(), RobotContainer.getDrive().getRotation(), config);
    } catch (Exception e) {
      return null;
    }
  }
}
