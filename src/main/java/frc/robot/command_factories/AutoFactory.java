package frc.robot.command_factories;


import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NemesisTimedCommand;
public class AutoFactory {
public static ArrayList<String> autoCommandNames= new ArrayList<>();



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


public static Command followPathdropL1(){


    try{

        PathPlannerPath path=
    }
}

public static Command holdThenScoreL() {

    
    RobotContainer.factoryCommands.add("HoldThenScoreL4");
    return ScoringFactory.scoreL4().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL4");
}

public static Command holdThenScoreL3() {
    RobotContainer.factoryCommands.add("HoldThenScoreL3");
    return ScoringFactory.scoreL3().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL3");
}

public static Command holdThenScoreL2() {
    RobotContainer.factoryCommands.add("HoldThenScoreL2");
    return ScoringFactory.scoreL2().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL2");
}
public static Command holdThenScoreL1() {
    RobotContainer.factoryCommands.add("HoldThenScoreL1");
    return ScoringFactory.scoreL1().onlyIf(()->Constants.locator.getZoneOfField(AutoBuilder.getCurrentPose()).equals("reef")).withName("HoldThenScoreL1");
}




// public static Command followPathThenAlign(String PathFileName){
//     followPath(PathFileName).andThen(Commands.run)
// }

}