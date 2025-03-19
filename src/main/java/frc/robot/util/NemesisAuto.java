package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class NemesisAuto {
    private static HashMap<String, Command> autoRoutines = new HashMap<String, Command>();

    public static void addAuto(String name, Command routine) {
        autoRoutines.put(name, routine);
    }

    public static SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        autoRoutines.forEach((name, routine) -> chooser.addOption(name, routine));
        return chooser;
    }
}
