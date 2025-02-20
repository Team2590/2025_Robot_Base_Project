package frc.robot.autos;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command_factories.AutoFactory;
import frc.robot.command_factories.AutoFactory.NemesisAuto;

public class AutoRoutines {
    
    public static LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    public static Command driveThenScoreL4= new AutoFactory.NemesisAuto("driveThenScoreL4", "preloadl1, source cycling", new String[]{"scoreL4","intake", "scoreL1"}).getCommand();
    


    public static void buildChooser(){

        autoChooser.addDefaultOption("driveScoreL4", driveThenScoreL4);
    }
    
}
