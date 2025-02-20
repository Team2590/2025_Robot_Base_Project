package frc.robot.autos;

import frc.robot.command_factories.AutoFactory;
import frc.robot.command_factories.AutoFactory.NemesisAuto;

public class AutoRoutines {


    public static NemesisAuto driveThenScoreL4= new AutoFactory.NemesisAuto("driveThenScoreL4", "preloadl1, source cycling", new String[]{"scoreL4","intake", "scoreL1"});
    
}
