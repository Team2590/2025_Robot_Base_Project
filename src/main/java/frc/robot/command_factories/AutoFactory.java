package frc.robot.command_factories;


import java.awt.Event;
import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.events.ScheduleCommandEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisTimedCommand;
public class AutoFactory {
public static ArrayList<String> autoCommandNames= new ArrayList<>();



public static RobotContainer container =Robot.getRobotContainerInstance();



public static Command followPath(String PathFileName){

    try{
    PathPlannerPath path=PathPlannerPath.fromPathFile(PathFileName);
        return AutoBuilder.followPath(path);
    }

    catch(Exception e){
    System.out.println("error in getting path" + e.getMessage());
    return Commands.print("oops, folowPath didn't work");
    }   
}


// public static Command followPathdropL1(){


//     try{

//         PathPlannerPath path=
//     }
// }

public static PathPlannerTrajectory getTrajectoryFromAuto(String autoFile, int index){
    RobotConfig config = container.getDrive().getConfig();
    try{
        PathPlannerPath p= PathPlannerAuto.getPathGroupFromAutoFile(autoFile).get(index);
        return p.generateTrajectory(new ChassisSpeeds(), container.getDrive().getRotation(), config);
    }
    catch( Exception e){
        return null;
    }
}



public static Command timedholdThenL4(String autoFile, Command x, int pathIndex, double timestamp){
    
    PathPlannerTrajectory chosenTrajectory = getTrajectoryFromAuto(autoFile, pathIndex);
    EventScheduler eventScheduler= new EventScheduler();
    Command raise= new Command() {
        
        public void initialize() {
            eventScheduler.initialize(chosenTrajectory);
            //EventTrigger.setCondition("inReef", ()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef"))
            //Trigger raiseEvent= new EventTrigger("scoreHere").onTrue(ScoringFactory.scoreL4());
            PointTowardsZoneTrigger zoneTrigger= new PointTowardsZoneTrigger("reefZone");
            //zoneTrigger.onTrue(ScoringFactory.scoreL4());

            ScheduleCommandEvent event = new ScheduleCommandEvent(timestamp, ScoringFactory.scoreL4());
            event.handleEvent(eventScheduler);

        }
        @Override
        public void execute() {
            eventScheduler.execute(0);
        }
        @Override
        public boolean isFinished() {
            return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("close");
        }
        
        


    };
    return raise;
    
    
}

public static Command holdThenL4= new Command() {
    int i;
    boolean finished;
    public void initialize() {

        i=0;
        finished=false;
        
    };
    public void execute() {
        if 
        (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && i==0){
            ScoringFactory.scoreL4().andThen(new InstantCommand(()->finished=true)).execute();
            i++;
        }



    };

    public boolean isFinished() {
        return finished;
    };

    public void end(boolean interrupted) {

        System.out.println("only run once robot immediately enters zone, then doesn't execute again");
    };
};

public static Command holdThenL3= new Command() {
    int i;
    boolean finished;
    public void initialize() {

        i=0;
        finished=false;
        
    };
    public void execute() {
        if 
        (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && i==0){
            ScoringFactory.scoreL3().andThen(new InstantCommand(()->finished=true)).execute();
            i++;
        }



    };

    public boolean isFinished() {
        return finished;
    };

    public void end(boolean interrupted) {

        System.out.println("only run once robot immediately enters zone, then doesn't execute again");
    };
};

public static Command holdThenL2= new Command() {
    int i;
    boolean finished;
    public void initialize() {

        i=0;
        finished=false;
        
    };
    public void execute() {
        if 
        (Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && i==0){
            ScoringFactory.scoreL2().andThen(new InstantCommand(()->finished=true)).execute();
            i++;
        }



    };

    public boolean isFinished() {
        return finished;
    };

    public void end(boolean interrupted) {

        System.out.println("only run once robot immediately enters zone, then doesn't execute again");
    };
};




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
        
// //         //ScoringFactory.scoreL4().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL4").execute();


// //     };
// //     public boolean isFinished() {
// //         return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();
        

        
// //     };

// //     public void end(boolean interrupted) {

// //         System.out.println("Command Done, stow while next driving");
// //     };
// // };




// public static Command holdThenL3= new Command() {
//     public void execute() {
        
//         ScoringFactory.scoreL3().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL3").execute();


//     };
//     public boolean isFinished() {
//         return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();
        

        
//     };
// };

// public static Command holdThenL2= new Command() {
//         public void execute() {
            
//             ScoringFactory.scoreL2().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScorel2").execute();
    
    
//         };
//         public boolean isFinished() {
//             return Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef") && ElevatorFactory.elevatorCommandFinished() && EndEffectorFactory.endEffectorCommandFinished();
            
    
            
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
// //     return ScoringFactory.scoreL4().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL4").repeatedly();
// // }

// // public static Command holdThenScoreL3() {
// //     //RobotContainer.factoryCommands.add("HoldThenScoreL3");
// //     return ScoringFactory.scoreL3().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL3");
// // }

// // public static Command holdThenScoreL2() {
// //     //RobotContainer.factoryCommands.add("HoldThenScoreL2");
// //     return ScoringFactory.scoreL2().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL2");
// // }
// // public static Command holdThenScoreL1() {
// //    // RobotContainer.factoryCommands.add("HoldThenScoreL1");
// //     return ScoringFactory.scoreL1().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL1");
// // }




// // public static Command followPathThenAlign(String PathFileName){
// //     followPath(PathFileName).andThen(Commands.run)
// // }

