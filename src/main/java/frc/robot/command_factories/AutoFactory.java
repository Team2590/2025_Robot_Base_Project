package frc.robot.command_factories;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.events.ScheduleCommandEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class AutoFactory {
  // public static ArrayList<String> autoCommandNames= new ArrayList<>();

  // public static RobotContainer container = Robot.getRobotContainerInstance();

  // public static RobotContainer container =Robot.getRobotContainerInstance();

  public static Command followPath(String PathFileName) {

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(PathFileName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("error in getting path" + e.getMessage());
      return Commands.print("oops, folowPath didn't work");
    }
  }

  // public static Command followPathdropL1(){

  //     try{

  //         PathPlannerPath path=
  //     }
  // }

  public static Command timedholdThenL4(
      String autoFile, Command x, int pathIndex, double timestamp) {

    PathPlannerTrajectory chosenTrajectory = getTrajectoryFromAuto(autoFile, pathIndex);
    EventScheduler eventScheduler = new EventScheduler();
    Command raise =
        new Command() {

          public void initialize() {
            eventScheduler.initialize(chosenTrajectory);
            // EventTrigger.setCondition("inReef",
            // ()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef"))
            // Trigger raiseEvent= new EventTrigger("scoreHere").onTrue(ScoringFactory.scoreL4());
            PointTowardsZoneTrigger zoneTrigger = new PointTowardsZoneTrigger("reefZone");
            // zoneTrigger.onTrue(ScoringFactory.scoreL4());

            ScheduleCommandEvent event =
                new ScheduleCommandEvent(timestamp, ScoringFactory.scoreL4());
            event.handleEvent(eventScheduler);
          }

          @Override
          public void execute() {
            eventScheduler.execute(0);
          }

          @Override
          public boolean isFinished() {
            return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
                && ElevatorFactory.elevatorCommandFinished()
                && EndEffectorFactory.endEffectorCommandFinished();
          }

          @Override
          public void end(boolean interrupted) {
            System.out.println("close");
          }
        };
    return raise;
  }

  public static Command holdThenL4 =
      new Command() {
        int i;
        boolean finished;

        public void initialize() {

          i = 0;
          finished = false;
        }
        ;

        public void execute() {
          if (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
              && i == 0) {
            ScoringFactory.scoreL4().andThen(new InstantCommand(() -> finished = true)).execute();
            i++;
          }
        }
        ;

        public boolean isFinished() {
          return finished;
        }
        ;

        public void end(boolean interrupted) {

          System.out.println(
              "only run once robot immediately enters zone, then doesn't execute again");
        }
        ;
      };

  public static Command holdThenL3 =
      new Command() {
        int i;
        boolean finished;

        public void initialize() {

          i = 0;
          finished = false;
        }
        ;

        public void execute() {
          if (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
              && i == 0) {
            ScoringFactory.scoreL3().andThen(new InstantCommand(() -> finished = true)).execute();
            i++;
          }
        }
        ;

        public boolean isFinished() {
          return finished;
        }
        ;

        public void end(boolean interrupted) {

          System.out.println(
              "only run once robot immediately enters zone, then doesn't execute again");
        }
        ;
      };

  public static Command holdThenL2 =
      new Command() {
        int i;
        boolean finished;

        public void initialize() {

          i = 0;
          finished = false;
        }
        ;

        public void execute() {
          if (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
              && i == 0) {
            ScoringFactory.scoreL2().andThen(new InstantCommand(() -> finished = true)).execute();
            i++;
          }
        }
        ;

        public boolean isFinished() {
          return finished;
        }
        ;

        public void end(boolean interrupted) {

          System.out.println(
              "only run once robot immediately enters zone, then doesn't execute again");
        }
        ;
      };

  public static int getIndexLinearSearch(List<String> elements, String name) {
    for (int i = 0; i < elements.size(); i++) {
      if (elements.get(i).equals(name)) {
        return i;
      }
    }
    return -1;
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

  public static class NemesisAuto {

    public String fileName;
    public String[] commandString;
    public PathPlannerAuto currentCommand;
    public String autoName;
    public PathPlannerAuto initialCommand;
    public boolean simMode;
    public ArrayList<Trigger> triggers;
    // public Command eventCommand;
    // Load in auto path Group from file

    /*
     * This constructor builds an auto that creates an EventScheduler and for each Trajectory it creates a command for that trajectory
     * So command1 will be for trajectory 1, command2 will be for trajectory2, command3 will be for trajectory 3 etc
     * When we run them all in parallel, hopefully when we are on trajectory1 command1 will run along with the pathfollowcommand, t
     *
     *
     */

    // goal, create  command that creates an eventscheduler for the current trajectory that the
    // robot is on, and uses that index to

    public NemesisAuto(String autoName, String fileName, String[] commandString, boolean simOrNah) {
      Command init = AutoBuilder.buildAuto(fileName);
      currentCommand = new PathPlannerAuto(init);
      simMode = simOrNah;

      List<PathPlannerPath> pathList;
      triggers = new ArrayList<>();
      try {
        pathList = PathPlannerAuto.getPathGroupFromAutoFile(fileName);
      } catch (Exception e) {
        pathList = null;
        System.out.println("oopsei with reading the . auto file " + e.getMessage());
      }
      // create conditionals for the auto, which is currently just pathFollowingCommands

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
      for (int i = 0; i < pathList.size(); i++) {
        Trigger whichPath =
            currentCommand.activePath(
                pathList.get(i)
                    .name); // creates a trigger thats true when we are on the currentPath of index
        // i

        // if the first path is being run, we want to use the first command in the command list
        // we are on the first path and the first command in the list, because if we want to do
        // something different while in the reef next time,
        Trigger inReef =
            currentCommand
                .condition(
                    () ->
                        Constants.locator
                            .getZoneOfField(AutoBuilder.getCurrentPose())
                            .equals("reef"))
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
        dumpL1.onTrue(ScoringFactory.scoreL1());

        Command chosen;
        // ['dumpl1 score'] will activate the Trigger for dumpL1 and intake based on their
        // conditions, this way on one path we can do multiple things
        String[] currentString = commandString[i].split(" ");
        for (String command : currentString) {

          switch (command) {
              // activate Triggers !
            default:
              chosen =
                  simMode
                      ? Commands.print("default commnad")
                      : Commands.print("mispelled or smth").andThen(ScoringFactory.stow());
              System.out.println("default switch case");

              break;
            case "none":
              chosen = Commands.none();
              break;
            case "scoreL4":
              chosen =
                  simMode
                      ? Commands.print(
                          "\n scoreL4 command placeholder") // on True this command should be
                      // scheduled
                      : ScoringFactory.scoreL4().andThen(ScoringFactory.stow());
              inReef.onTrue(chosen);
              System.out.println(" \n  l4 switch case, Trigger created     \n");
              break;
            case "scoreL1":
              chosen =
                  simMode
                      ? Commands.print("scoreL1 command placeholder")
                      : ScoringFactory.scoreL1().andThen(ScoringFactory.stow());

              inReef.onTrue(chosen);
              break;
            case "dumpL1":
              chosen = ScoringFactory.scoreL1();
              dumpL1.onTrue(chosen);

            case "intakeFromStation":
              chosen =
                  simMode
                      ? Commands.print("intake command placeholder")
                      : EndEffectorFactory.runEndEffector().andThen(ScoringFactory.stow());

              atIntakeStation.onTrue(chosen);

              break;

            case "processor":
              chosen =
                  simMode
                      ? Commands.print("processor command placeholder")
                      : IntakeFactory.setHoldingAlgaePosition();
              scoreProcessor.onTrue(chosen);
          }

          triggers.add(inReef);
          triggers.add(whichPath);
          triggers.add(atIntakeStation);
          triggers.add(scoreProcessor);
          triggers.add(dumpL1);
        }
        // populate the triggers to work

      }
    }

    public Command getCommand() {
      return currentCommand;
    }

    public ArrayList<Trigger> getTriggers() {
      return triggers;
    }
  }

  public static void logAutoCommandsSim(String message) {
    Drive.autoCommandMessage = message;
  }
}

// }
// // public static Command holdThenL4= new Command() {

// //     public void initialize() {
// //         RobotConfig config = container.getDrive().getConfig();

// //         EventScheduler scheduler= new EventScheduler();
// //         scheduler.initialize( getTrajectoryFromAuto("", 0));
// //     };

// //     public void execute() {
// //         if(Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")){
// //             ScoringFactory.scoreL4().execute();
// //         }

// //
// //ScoringFactory.scoreL4().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL4").execute();

// //     };
// //     public boolean isFinished() {
// //         return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
// && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();

// //     };

// //     public void end(boolean interrupted) {

// //         System.out.println("Command Done, stow while next driving");
// //     };
// // };

// public static Command holdThenL3= new Command() {
//     public void execute() {

//
// ScoringFactory.scoreL3().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL3").execute();

//     };
//     public boolean isFinished() {
//         return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") &&
// ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();

//     };
// };

// public static Command holdThenL2= new Command() {
//         public void execute() {

//
// ScoringFactory.scoreL2().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScorel2").execute();

//         };
//         public boolean isFinished() {
//             return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")
// && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();

//         };

//         public void end(boolean interrupted) {

//             System.out.println("Command Done, stow while next driving");
//         };
//     };

//     public void end(boolean interrupted) {

//         System.out.println("Command Done, stow while next driving");
//     };

// };

// // public static Command holdThenScoreL4() {

// //    // RobotContainer.factoryCommands.add("HoldThenScoreL4");
// //     return
// ScoringFactory.scoreL4().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL4").repeatedly();
// // }

// // public static Command holdThenScoreL3() {
// //     //RobotContainer.factoryCommands.add("HoldThenScoreL3");
// //     return
// ScoringFactory.scoreL3().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL3");
// // }

// // public static Command holdThenScoreL2() {
// //     //RobotContainer.factoryCommands.add("HoldThenScoreL2");
// //     return
// ScoringFactory.scoreL2().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL2");
// // }
// // public static Command holdThenScoreL1() {
// //    // RobotContainer.factoryCommands.add("HoldThenScoreL1");
// //     return
// ScoringFactory.scoreL1().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL1");
// // }

// // public static Command followPathThenAlign(String PathFileName){
// //     followPath(PathFileName).andThen(Commands.run)
// // }
